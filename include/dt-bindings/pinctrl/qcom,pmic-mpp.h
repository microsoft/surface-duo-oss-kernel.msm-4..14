/*
 * This header provides constants for the Qualcomm PMIC's
 * Multi-Purpose Pin binding.
 */

#ifndef _DT_BINDINGS_PINCTRL_QCOM_PMIC_MPP_H
#define _DT_BINDINGS_PINCTRL_QCOM_PMIC_MPP_H

/* power-source */
#define PM8841_MPP_VPH			0
#define PM8841_MPP_S3			2

#define PM8941_MPP_VPH			0
#define PM8941_MPP_L1			1
#define PM8941_MPP_S3			2
#define PM8941_MPP_L6			3

#define PMA8084_MPP_VPH			0
#define PMA8084_MPP_L1			1
#define PMA8084_MPP_S4			2
#define PMA8084_MPP_L6			3

/*
 * Analog Output - Set the analog output voltage reference.
 * To be used with "qcom,vrefence = <>"
 */
#define PMIC_MPP_VREFERENCE_1V25	0
#define PMIC_MPP_VREFERENCE_0V625	1
#define PMIC_MPP_VREFERENCE_0V3125	2
#define PMIC_MPP_VREFERENCE_PAIRED_MPP	3
#define PMIC_MPP_VREFERENCE_ABUS1	4
#define PMIC_MPP_VREFERENCE_ABUS2	5
#define PMIC_MPP_VREFERENCE_ABUS3	6
#define PMIC_MPP_VREFERENCE_ABUS4	7

/*
 * Analog Input - Set the source for analog input.
 * To be used with "qcom,amux-route = <>"
 */
#define PMIC_MPP_AMUX_ROUTE_CH5		0
#define PMIC_MPP_AMUX_ROUTE_CH6		1
#define PMIC_MPP_AMUX_ROUTE_CH7		2
#define PMIC_MPP_AMUX_ROUTE_CH8		3
#define PMIC_MPP_AMUX_ROUTE_ABUS1	4
#define PMIC_MPP_AMUX_ROUTE_ABUS2	5
#define PMIC_MPP_AMUX_ROUTE_ABUS3	6
#define PMIC_MPP_AMUX_ROUTE_ABUS4	7

/*
 * Mode select - indicates whether the pin should be digital input, output, both
 * or analog input, output or current sink. To be used with "qcom,mode = <>"
 */
#define PMIC_MPP_MODE_DI		0
#define PMIC_MPP_MODE_DO		1
#define PMIC_MPP_MODE_DIO		2
#define PMIC_MPP_MODE_AIO		3
#define PMIC_MPP_MODE_AI		4
#define PMIC_MPP_MODE_AO		5
#define PMIC_MPP_MODE_CS		6

/* To be used with "function = " */
#define PMIC_MPP_FUNC_NORMAL		"normal"
#define PMIC_MPP_FUNC_PAIRED		"paired"
#define PMIC_MPP_FUNC_DTEST1		"dtest1"
#define PMIC_MPP_FUNC_DTEST2		"dtest2"
#define PMIC_MPP_FUNC_DTEST3		"dtest3"
#define PMIC_MPP_FUNC_DTEST4		"dtest4"

#endif

