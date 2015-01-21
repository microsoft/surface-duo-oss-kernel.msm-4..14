obj-$(CONFIG_QCOM_GSBI)	+=	qcom_gsbi.o
obj-$(CONFIG_QCOM_PM)	+=	spm.o
obj-$(CONFIG_MSM_PIL)	+=	peripheral-loader.o
CFLAGS_scm.o :=$(call as-instr,.arch_extension sec,-DREQUIRES_SEC=1)
obj-$(CONFIG_QCOM_SCM) += scm.o scm-boot.o scm-pas.o
