/*
 * Copyright 2015 Linaro Limited
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _DT_BINDINGS_CLK_MSM_RPMCC_H
#define _DT_BINDINGS_CLK_MSM_RPMCC_H

/* apq8064 */
#define RPM_PXO_CLK				0
#define RPM_PXO_A_CLK				1
#define RPM_CXO_CLK				2
#define RPM_CXO_A_CLK				3
#define RPM_APPS_FABRIC_CLK			4
#define RPM_APPS_FABRIC_A_CLK			5
#define RPM_CFPB_CLK				6
#define RPM_CFPB_A_CLK				7
#define RPM_QDSS_CLK				8
#define RPM_QDSS_A_CLK				9
#define RPM_DAYTONA_FABRIC_CLK			10
#define RPM_DAYTONA_FABRIC_A_CLK		11
#define RPM_EBI1_CLK				12
#define RPM_EBI1_A_CLK				13
#define RPM_MM_FABRIC_CLK			14
#define RPM_MM_FABRIC_A_CLK			15
#define RPM_MMFPB_CLK				16
#define RPM_MMFPB_A_CLK				17
#define RPM_SYS_FABRIC_CLK			18
#define RPM_SYS_FABRIC_A_CLK			19
#define RPM_SFPB_CLK				20
#define RPM_SFPB_A_CLK				21

/* msm8916 */
#define RPM_SMD_XO_CLK_SRC				0
#define RPM_SMD_XO_A_CLK_SRC			1
#define RPM_SMD_PCNOC_CLK				2
#define RPM_SMD_PCNOC_A_CLK				3
#define RPM_SMD_SNOC_CLK				4
#define RPM_SMD_SNOC_A_CLK				5
#define RPM_SMD_BIMC_CLK				6
#define RPM_SMD_BIMC_A_CLK				7
#define RPM_SMD_QDSS_CLK				8
#define RPM_SMD_QDSS_A_CLK				9
#define RPM_SMD_BB_CLK1				10
#define RPM_SMD_BB_CLK1_A				11
#define RPM_SMD_BB_CLK2				12
#define RPM_SMD_BB_CLK2_A				13
#define RPM_SMD_RF_CLK1				14
#define RPM_SMD_RF_CLK1_A				15
#define RPM_SMD_RF_CLK2				16
#define RPM_SMD_RF_CLK2_A				17
#define RPM_SMD_BB_CLK1_PIN				18
#define RPM_SMD_BB_CLK1_A_PIN			19
#define RPM_SMD_BB_CLK2_PIN				20
#define RPM_SMD_BB_CLK2_A_PIN			21
#define RPM_SMD_RF_CLK1_PIN				22
#define RPM_SMD_RF_CLK1_A_PIN			23
#define RPM_SMD_RF_CLK2_PIN				24
#define RPM_SMD_RF_CLK2_A_PIN			25

/* msm8996 */
#define MSM8996_RPM_SMD_XO_CLK_SRC				0
#define MSM8996_RPM_SMD_XO_A_CLK_SRC			1
#define MSM8996_RPM_SMD_PCNOC_CLK				2
#define MSM8996_RPM_SMD_PCNOC_A_CLK				3
#define MSM8996_RPM_SMD_SNOC_CLK				4
#define MSM8996_RPM_SMD_SNOC_A_CLK				5
#define MSM8996_RPM_SMD_BIMC_CLK				6
#define MSM8996_RPM_SMD_BIMC_A_CLK				7
#define MSM8996_RPM_SMD_QDSS_CLK				8
#define MSM8996_RPM_SMD_QDSS_A_CLK				9
#define MSM8996_RPM_SMD_BB_CLK1				10
#define MSM8996_RPM_SMD_BB_CLK1_A				11
#define MSM8996_RPM_SMD_BB_CLK2				12
#define MSM8996_RPM_SMD_BB_CLK2_A				13
#define MSM8996_RPM_SMD_RF_CLK1				14
#define MSM8996_RPM_SMD_RF_CLK1_A				15
#define MSM8996_RPM_SMD_RF_CLK2				16
#define MSM8996_RPM_SMD_RF_CLK2_A				17
#define MSM8996_RPM_SMD_BB_CLK1_PIN				18
#define MSM8996_RPM_SMD_BB_CLK1_A_PIN			19
#define MSM8996_RPM_SMD_BB_CLK2_PIN				20
#define MSM8996_RPM_SMD_BB_CLK2_A_PIN			21
#define MSM8996_RPM_SMD_RF_CLK1_PIN				22
#define MSM8996_RPM_SMD_RF_CLK1_A_PIN			23
#define MSM8996_RPM_SMD_RF_CLK2_PIN				24
#define MSM8996_RPM_SMD_RF_CLK2_A_PIN			25
#define MSM8996_RPM_SMD_AGGR1_NOC_CLK			26
#define MSM8996_RPM_SMD_AGGR1_NOC_A_CLK			27
#define MSM8996_RPM_SMD_AGGR2_NOC_CLK			28
#define MSM8996_RPM_SMD_AGGR2_NOC_A_CLK			29
#define MSM8996_RPM_SMD_CNOC_CLK				30
#define MSM8996_RPM_SMD_CNOC_A_CLK				31
#define MSM8996_RPM_SMD_MMAXI_CLK				32
#define MSM8996_RPM_SMD_MMAXI_A_CLK				33
#define MSM8996_RPM_SMD_IPA_CLK				34
#define MSM8996_RPM_SMD_IPA_A_CLK				35
#define MSM8996_RPM_SMD_CE1_CLK				36
#define MSM8996_RPM_SMD_CE1_A_CLK				37
#define MSM8996_RPM_SMD_DIV_CLK1				38
#define MSM8996_RPM_SMD_DIV_CLK1_AO				39
#define MSM8996_RPM_SMD_DIV_CLK2				40
#define MSM8996_RPM_SMD_DIV_CLK2_AO				41
#define MSM8996_RPM_SMD_DIV_CLK3				42
#define MSM8996_RPM_SMD_DIV_CLK3_AO				43
#define MSM8996_RPM_SMD_LN_BB_CLK				44
#define MSM8996_RPM_SMD_LN_BB_A_CLK				45

/* msm8974 */
#define RPM_SMD_CXO_CLK_SRC                                0
#define RPM_SMD_CXO_A_CLK_SRC                      1
#define RPM_SMD_PNOC_CLK                           2
#define RPM_SMD_PNOC_A_CLK                         3
#define RPM_SMD_SNOC_CLK                           4
#define RPM_SMD_SNOC_A_CLK                         5
#define RPM_SMD_BIMC_CLK                           6
#define RPM_SMD_BIMC_A_CLK                         7
#define RPM_SMD_QDSS_CLK                           8
#define RPM_SMD_QDSS_A_CLK                         9
#define RPM_SMD_CXO_D0                             10
#define RPM_SMD_CXO_D0_A                           11
#define RPM_SMD_CXO_D1                             12
#define RPM_SMD_CXO_D1_A                           13
#define RPM_SMD_CXO_A0                             14
#define RPM_SMD_CXO_A0_A                           15
#define RPM_SMD_CXO_A1                             16
#define RPM_SMD_CXO_A1_A                           17
#define RPM_SMD_CXO_A2                             18
#define RPM_SMD_CXO_A2_A                           19
#define RPM_SMD_CXO_D0_PIN                         20
#define RPM_SMD_CXO_D0_A_PIN                       21
#define RPM_SMD_CXO_A2_PIN                         22
#define RPM_SMD_CXO_A2_A_PIN                       23
#define RPM_SMD_CXO_A1_PIN                         24
#define RPM_SMD_CXO_A1_A_PIN                       25
#define RPM_SMD_DIFF_CLK                           26
#define RPM_SMD_DIFF_A_CLK                         27
#define RPM_SMD_CNOC_CLK                           28
#define RPM_SMD_CNOC_A_CLK                         29
#define RPM_SMD_MMSSNOC_AHB_CLK                    30
#define RPM_SMD_MMSSNOC_AHB_A_CLK                  31
#define RPM_SMD_OCMEMGX_CLK                         32
#define RPM_SMD_OCMEMGX_A_CLK                      33
#define RPM_SMD_GFX3D_CLK_SRC                      34
#define RPM_SMD_GFX3D_A_CLK_SRC                    35
#define RPM_SMD_DIV_CLK1                           36
#define RPM_SMD_DIV_A_CLK1                         37
#define RPM_SMD_DIV_CLK2                           38
#define RPM_SMD_DIV_A_CLK2                         39
#define RPM_SMD_CXO_D1_PIN                         40
#define RPM_SMD_CXO_D1_A_PIN                       41
#define RPM_SMD_CXO_A0_PIN                         42
#define RPM_SMD_CXO_A0_A_PIN                       43

/* apq8084 */
#define RPM_SMD_XO_CLK_SRC				0
#define RPM_SMD_XO_A_CLK_SRC			1
#define RPM_SMD_PNOC_CLK				2
#define RPM_SMD_PNOC_A_CLK				3
#define RPM_SMD_SNOC_CLK				4
#define RPM_SMD_SNOC_A_CLK				5
#define RPM_SMD_BIMC_CLK				6
#define RPM_SMD_BIMC_A_CLK				7
#define RPM_SMD_QDSS_CLK				8
#define RPM_SMD_QDSS_A_CLK				9
#define RPM_SMD_BB_CLK1				10
#define RPM_SMD_BB_CLK1_A				11
#define RPM_SMD_BB_CLK2				12
#define RPM_SMD_BB_CLK2_A				13
#define RPM_SMD_RF_CLK1				14
#define RPM_SMD_RF_CLK1_A				15
#define RPM_SMD_RF_CLK2				16
#define RPM_SMD_RF_CLK2_A				17
#define RPM_SMD_BB_CLK1_PIN				18
#define RPM_SMD_BB_CLK1_A_PIN			19
#define RPM_SMD_BB_CLK2_PIN				20
#define RPM_SMD_BB_CLK2_A_PIN			21
#define RPM_SMD_RF_CLK1_PIN				22
#define RPM_SMD_RF_CLK1_A_PIN			23
#define RPM_SMD_RF_CLK2_PIN				24
#define RPM_SMD_RF_CLK2_A_PIN			25
#define RPM_SMD_DIFF_CLK1				26
#define RPM_SMD_DIFF_CLK1_A				27
#define RPM_SMD_CNOC_CLK				28
#define RPM_SMD_CNOC_A_CLK				29
#define RPM_SMD_MMSSNOC_AHB_CLK			30
#define RPM_SMD_MMSSNOC_AHB_A_CLK			31
#define RPM_SMD_OCMEMGX_CLK				32
#define RPM_SMD_OCMEMGX_A_CLK			33
#define RPM_SMD_GFX3D_CLK_SRC			34
#define RPM_SMD_GFX3D_A_CLK_SRC			35
#define RPM_SMD_DIV_CLK1				36
#define RPM_SMD_DIV_CLK1_A				37
#define RPM_SMD_DIV_CLK2				38
#define RPM_SMD_DIV_CLK2_A				39
#define RPM_SMD_DIV_CLK3				40
#define RPM_SMD_DIV_CLK3_A				41
#define RPM_SMD_RF_CLK3_PIN				44
#define RPM_SMD_RF_CLK3_A_PIN			45
#define RPM_SMD_RF_CLK3				46
#define RPM_SMD_RF_CLK3_A				47

#endif
