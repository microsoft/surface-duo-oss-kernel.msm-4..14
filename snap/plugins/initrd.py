# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018,2020 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""The initrd plugin is heavily inspired by kernel plugin
focusing on initrd content

WARNING: this plugin's API is unstable. The cross compiling support is
         experimental.

The following initrd specific options are provided by this plugin:

    - initrd-modules:
      (array of string)
      list of modules to include in initrd; note that kernel snaps do not
      provide the core boot logic which comes from snappy Ubuntu Core
      OS snap. Include all modules you need for mounting rootfs here.

    - initrd-firmware:
      (array of string)
      list of firmware files to include in the initrd; these need to be
      relative paths to stage directory.

    - initrd-compression:
      (string; default: lz4)
      initrd compression to use; the only supported values now are 'lz4', 'xz', 'gz'.

    - initrd-compression-options:
      Optional list of parameters to be passed to compressor used for initrd
      (array of string): defaults are
        gz:  -7
        lz4: -9 -l
        xz:  -7

    - initrd-flavour
      Optional parameter(Default flavour is none). For uc16/18 supported flavour is 'fde'

    - initrd-base-url
      Optional base url to be used to download reference inird from.
      TODO: this will eventually override snap store download option
      For now this overides base url from default people.canonical.com/~okubik/...
      Default: https://people.canonical.com/~okubik/uc-initrds

    - initrd-overlay
      Optional overlay to be applied to built initrd
      This option is designed to provide easy way to apply initrd overlay for cases
      as full disk encryption support, when device specific hooks need to be added
      to the initrd.
      Value is relative path, in stage directory. and related part needs to be
      built before initrd part. During build it will be expanded to
      ${SNAPCRAFT_STAGE}/{initrd-overlay}
      Default: none
    - initrd-core-base
      Optional override to specify target Ubuntu Core base, regardless if there
      is defined build-base in the top level of the snapcraft project.
      Supported values are same as for build-base: 'core(16)', 'core18', 'core20'.
"""

import glob
import logging
import os
import shutil
import subprocess
import urllib.parse

import snapcraft
from snapcraft.plugins.v1 import PluginV1
from os import listdir
from snapcraft.internal.indicators import (
     download_urllib_source,
)
from snapcraft.internal import errors
from snapcraft.internal.errors import SnapcraftPluginCommandError

logger = logging.getLogger(__name__)

_compression_command = {"gz": "gzip", "lz4":"lz4", "xz": "xz"}
_compressor_options = {"gz": "-7", "lz4": "-l -9", "xz": "-7" }
_INITRD_BASE_URL = "https://people.canonical.com/~okubik/uc-initrds"
_INITRD_URL  = "{base_url}/{snap_name}"
_INITRD_SNAP_NAME = "uc-initrd"
_INITRD_SNAP_FILE = "{snap_name}_{series}{flavour}_{architecture}.snap"

required_boot = ["squashfs"]


class InitrdPlugin(PluginV1):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["initrd-modules"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["initrd-firmware"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["initrd-compression"] = {
            "type": "string",
            "default": "lz4",
            "enum": ["lz4"],
        }

        schema["properties"]["initrd-compression-options"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["initrd-flavour"] = {
            "type": "string",
            "default": ""
        }

        schema["properties"]["initrd-base-url"] = {
            "type": "string",
            "default": ""
        }

        schema["properties"]["initrd-overlay"] = {
            "type": "string",
            "default": ""
        }

        schema["properties"]["initrd-core-base"] = {
            "type": "string",
            "default": ""
        }

        return schema

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return super().get_build_properties() + [
            "initrd-modules",
            "initrd-firmware",
            "initrd-compression",
            "initrd-compression-options",
            "initrd-flavour",
            "initrd-base-url",
            "initrd-overlay",
            "initrd-core-base"
        ]

    @property
    def compression_cmd(self):
        compressor = _compression_command[self.options.initrd_compression]
        options = ""
        if self.options.initrd_compression_options:
            for opt in self.options.initrd_compression_options:
                options = "{} {}".format(options, opt)
        else:
            options = _compressor_options[self.options.initrd_compression]

        cmd = "{} {}".format(compressor, options)
        logger.info(
            "Using initrd compressions command: {!r}".format(cmd)
        )
        return cmd

    def __init__(self, name, options, project):

        super().__init__(name, options, project)

        # We need to be able to shell out to modprobe
        self.build_packages.append("kmod")
        # add compression tools
        self.build_packages.append("xz-utils")
        # we cannot assume there is lsb_release, parse it manually
        with open("/etc/lsb-release") as lsb_file:
            for line in lsb_file.read().splitlines():
                if "DISTRIB_CODENAME" in line:
                    lsb_release = line.split("=")[1]
                    break

        if lsb_release == "focal":
            self.build_packages.append("lz4")
        else:
            self.build_packages.append("liblz4-tool")

        self.kernel_release = ""
        self._setup_base(project._get_build_base())

        if self.project.target_arch is not None:
            self.initrd_arch = self.project.target_arch
        else:
            self.initrd_arch = self.project.kernel_arch

        if self.options.initrd_flavour:
            flavour = "-{}".format(self.options.initrd_flavour)
        else:
            flavour = ""

        # determine type of initrd
        initrd_snap_file_name = _INITRD_SNAP_FILE.format(
            snap_name=_INITRD_SNAP_NAME,
            series=self.uc_series,
            flavour=flavour,
            architecture=self.initrd_arch
        )
        if self.options.initrd_base_url:
            self.snap_url = _INITRD_URL.format(
                base_url=self.options.initrd_base_url,
                snap_name=initrd_snap_file_name
            )
        else:
            self.snap_url = _INITRD_URL.format(
                base_url=_INITRD_BASE_URL,
                snap_name=initrd_snap_file_name
            )

        self.vanilla_initrd_snap = os.path.join(self.sourcedir,
            initrd_snap_file_name
        )

    def enable_cross_compilation(self):
        logger.info(
			"Cross compiling initrd to {!r}".format(self.project.kernel_arch)
        )
        pass


    def _unpack_generic_initrd(self):
        initrd_path = os.path.join("initrd.img")
        initrd_unpacked_path = os.path.join(self.builddir, "initrd-staging")
        initrd_unpacked_snap = os.path.join(self.builddir, "unpacked_snap")
        if os.path.exists(initrd_unpacked_path):
            shutil.rmtree(initrd_unpacked_path)
        os.makedirs(initrd_unpacked_path)

        unsquashfs_path = snapcraft.file_utils.get_snap_tool_path("unsquashfs")
        subprocess.check_call(
            [
                unsquashfs_path,
                "-f",
                "-d",
                initrd_unpacked_snap,
                self.vanilla_initrd_snap
            ],
            cwd=self.builddir,
        )
        tmp_initrd_path = os.path.join(initrd_unpacked_snap, initrd_path)

        decompressed = False
        # Roll over valid decompression mechanisms until one works
        for decompressor in ("xz", "gzip", "lz4"):
            try:
                subprocess.check_call(
                    "cat {0} | {1} -dc | cpio -id".format(
                        tmp_initrd_path, decompressor
                        ),
                        shell=True,
                        cwd=initrd_unpacked_path,
                )
            except subprocess.CalledProcessError:
                pass
            else:
                decompressed = True
                break

        if not decompressed:
            raise RuntimeError("The initrd file type is unsupported")

        return initrd_unpacked_path

    def _make_initrd(self):

        logger.info(
            "Generating initrd with ko modules for kernel release: {}".format(
                self.kernel_release
            )
        )

        initrd_unpacked_path = self._unpack_generic_initrd()
        initrd_modules_dir = os.path.join(
            initrd_unpacked_path,
            "lib",
            "modules",
            self.kernel_release
        )
        os.makedirs(initrd_modules_dir)

        modprobe_outs = []

        # modprobe is typically in /sbin, which may not always be available on
        # the PATH. Add it for this call.
        env = os.environ.copy()
        env["PATH"] += ":/sbin"
        for module in self.options.initrd_modules:
            try:
                modprobe_out = self.run_output(
                    [
                        "modprobe",
                        "-n",
                        "-q",
                        "--show-depends",
                        "-d",
                        self.project.stage_dir,
                        "-S",
                        self.kernel_release,
                        module,
                    ],
                    env=env
                )
            except errors.SnapcraftPluginCommandError:
                logger.warning("**** WARNING **** Cannot find module '{}', ignoring it!!!".format(module))
            else:
                modprobe_outs.extend(modprobe_out.split(os.linesep))

        modprobe_outs = [_ for _ in modprobe_outs if _]
        modules_path = os.path.join("lib", "modules", self.kernel_release)
        for module_info in modprobe_outs:
            type = module_info.split()[0]
            src = module_info.split()[1]
            if type == "builtin":
                logger.info(
                    "Module '{}' is built in, ignoring it".format(src)
                )
                continue

            dst = os.path.join(
                initrd_unpacked_path,
                os.path.relpath(
                    src,
                    self.project.stage_dir
                )
            )
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            if os.path.isfile(src):
                self._link_replace(src, dst)
            else:
                logger.warning(
                    "**** WARNING **** Cannot locate dependency '{}' for included modules!!! **** WARNING ****".format(src)
                )

        if modprobe_outs:
            for module_info in os.listdir(os.path.join(self.project.stage_dir,modules_path)):
                module_info_path = os.path.join(modules_path, module_info)
                src = os.path.join(self.project.stage_dir, module_info_path)
                dst = os.path.join(initrd_unpacked_path, module_info_path)
                # ignore directories
                if not os.path.isdir(src):
                    self._link_replace(src, dst)

        # update module dependencies
        subprocess.check_call(
            "depmod -b {} {}".format(initrd_unpacked_path, self.kernel_release),
            shell=True,
            cwd=initrd_unpacked_path,
        )

        # gather firmware files
        for firmware in self.options.initrd_firmware:
            # firmware can be from stage pakage as part of this part or from stage
            # firmware from initrd part stage packages takes preference
            src = os.path.join(self.installdir, firmware)
            if not os.path.exists(src):
                src = os.path.join(self.project.stage_dir, firmware)
            dst = os.path.join(initrd_unpacked_path, "lib", firmware)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            if os.path.isdir(src):
                shutil.copytree(src, dst)
            else:
                self._link_replace(src, dst)

        # apply overlay if defined
        if self.options.initrd_overlay:
            overlay_src = os.path.join(
                self.project.stage_dir,
                self.options.initrd_overlay
            )
            shutil.copytree(overlay_src, initrd_unpacked_path)

        initrd = "initrd-{}.img".format(self.kernel_release)
        initrd_path = os.path.join(self.installdir, initrd)
        subprocess.check_call(
            "find . | cpio --create --format=newc --owner=0:0 | "
            "{} > {}".format(self.compression_cmd, initrd_path),
            shell=True,
            cwd=initrd_unpacked_path,
        )
        unversioned_initrd_path = os.path.join(self.installdir, "initrd.img")
        self._link_replace(initrd_path, unversioned_initrd_path)

    def _parse_kernel_release(self):
        modules_path = os.path.join(
            self.project.stage_dir, "modules"
        )

        for f in listdir(modules_path):
            self.kernel_release = f[f.find("-")+1:]

        if not self.kernel_release:
            raise ValueError(
                "No kernel release version info found at {!r}".format(
                    kernel_release_path
                )
            )

    def _do_parse_config(self, config_path):
        builtin = []
        modules = []
        # tokenize .config and store options in builtin[] or modules[]
        with open(config_path) as f:
            for line in f:
                tok = line.strip().split("=")
                items = len(tok)
                if items == 2:
                    opt = tok[0].upper()
                    val = tok[1].upper()
                    if val == "Y":
                        builtin.append(opt)
                    elif val == "M":
                        modules.append(opt)
        return builtin, modules


    def _do_check_initrd(self, builtin, modules):
        # check all required_boot[] items are either builtin or part of initrd
        msg = (
            "**** WARNING **** WARNING **** WARNING **** WARNING ****\n"
            "The following features are deemed boot essential for\n"
            "ubuntu core, consider making them static[=Y] or adding\n"
            "the corresponding module to initrd:\n"
        )
        missing = []

        for code in required_boot:
            opt = "CONFIG_{}".format(code.upper())
            if opt in builtin:
                continue
            elif opt in modules:
                if code in self.options.initrd_modules:
                    continue
                else:
                    missing.append(opt)
            else:
                missing.append(opt)

        if missing:
            warn = "\n{}\n".format(msg)
            for opt in missing:
                warn += "{}\n".format(opt)
            logger.warning(warn)

    def _generate_module_dep(self):
        self.run(
                [
                    "depmod",
                    "-b",
                    self.project.stage_dir,
                    "-F",
                    os.path.join(
                        self.project.stage_dir,
                        "System.map-{}".format(self.kernel_release)
                    ),
                    "-w",
                    self.kernel_release,
                ]
        )

    def _setup_base(self, base):
        # if base overide is defined, use it instead
        if self.options.initrd_core_base:
            build_base = self.options.initrd_core_base
        else:
            build_base = base

        if build_base in ("core", "core16"):
            self.uc_series = "16"
        elif build_base == "core18":
            self.uc_series = "18"
        elif build_base == "core20":
            eslf.uc_series = "20"
        else:
            raise errors.PluginBaseError(part_name=self.name, base=build_base)

    # link source and destination, replacing destination if it exists
    def _link_replace(self, src, dst):
        if os.path.exists(dst):
            os.remove(dst)
        os.link(src, dst)

    def pull(self):
        logger.info("Using reference initrd: {}".format(self.snap_url))
        is_source_url = snapcraft.internal.common.isurl(self.snap_url)
        # TODO: this should be eventually pulled from snap store
        # for now check if url is valid and use it
        # If not we try to download it from store
        if is_source_url:
            download_urllib_source(self.snap_url, self.vanilla_initrd_snap)
        else:
            snapcraft.download(
                _INITRD_SNAP_NAME,
                risk="stable",
                track=self.uc_series,
                download_path=self.vanilla_initrd_snap,
                arch=self.initrd_arch
            )

    def do_configure(self):
        builtin, modules = self._do_parse_config(
            os.path.join(
                self.project.stage_dir,
                "config-{}".format(
                    self.kernel_release
                )
            )
        )
        self._do_check_initrd(builtin, modules)

    def build(self):
        super().build()

        self._parse_kernel_release()
        self.do_configure()
        self._generate_module_dep()
        self._make_initrd()
