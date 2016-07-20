# Platform
snd-soc-lpass-cpu-objs := lpass-cpu.o
snd-soc-lpass-platform-objs := lpass-platform.o
snd-soc-lpass-ipq806x-objs := lpass-ipq806x.o
snd-soc-lpass-apq8016-objs := lpass-apq8016.o

obj-$(CONFIG_SND_SOC_LPASS_CPU) += snd-soc-lpass-cpu.o
obj-$(CONFIG_SND_SOC_LPASS_PLATFORM) += snd-soc-lpass-platform.o
obj-$(CONFIG_SND_SOC_LPASS_IPQ806X) += snd-soc-lpass-ipq806x.o
obj-$(CONFIG_SND_SOC_LPASS_APQ8016) += snd-soc-lpass-apq8016.o

# Machine
snd-soc-storm-objs := storm.o
snd-soc-apq8016-sbc-objs := apq8016_sbc.o

obj-$(CONFIG_SND_SOC_STORM) += snd-soc-storm.o
obj-$(CONFIG_SND_SOC_APQ8016_SBC) += snd-soc-apq8016-sbc.o

# for MSM 8960 sound card driver
obj-$(CONFIG_SND_SOC_MSM_QDSP6_INTF) += qdsp6/
snd-soc-qdsp6-objs :=  msm-pcm-q6.o msm-pcm-routing.o msm-dai-fe.o 
obj-$(CONFIG_SND_SOC_MSM_QDSP6_HDMI_AUDIO) += msm-dai-q6-hdmi.o
obj-$(CONFIG_SND_SOC_QDSP6) += snd-soc-qdsp6.o

snd-soc-msm8960-objs := apq8064.o 
obj-$(CONFIG_SND_SOC_MSM8960) += snd-soc-msm8960.o
