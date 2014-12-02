obj-$(CONFIG_QCOM_GSBI)	+=	qcom_gsbi.o
obj-$(CONFIG_QCOM_PM)	+=	spm.o
CFLAGS_scm.o :=$(call as-instr,.arch_extension sec,-DREQUIRES_SEC=1)
obj-$(CONFIG_QCOM_SCM) += scm.o scm-boot.o
