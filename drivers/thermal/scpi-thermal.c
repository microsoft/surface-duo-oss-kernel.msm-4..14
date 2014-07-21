#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_actor.h>
#include <linux/scpi_protocol.h>
#include <linux/thermal.h>
#include <linux/topology.h>

#define SOC_SENSOR "SENSOR_TEMP_SOC"

#define NUM_CLUSTERS 2
enum cluster_type {
	CLUSTER_BIG = 0,
	CLUSTER_LITTLE
};

struct scpi_sensor {
	u16 sensor_id;
	struct thermal_zone_device *tzd;
	struct cpumask cluster[NUM_CLUSTERS];
	struct power_actor *pactor[NUM_CLUSTERS];
};

struct scpi_sensor scpi_temp_sensor;

static int get_dyn_power_coeff(enum cluster_type cluster)
{
	int coeff = 0;

	switch(cluster) {
	case CLUSTER_BIG:
		coeff = 530;
		break;
	case CLUSTER_LITTLE:
		coeff = 140;
		break;
	}

	return coeff;
}

static int get_cpu_static_power_coeff(enum cluster_type cluster)
{
	int coeff = 0;

	switch(cluster) {
	case CLUSTER_BIG:
		/* 75mW @ 85C/0.9V */
		coeff = 103;
		break;
	case CLUSTER_LITTLE:
		/* 26mW @ 85C/0.9V */
		coeff = 36;
		break;
	}

	return coeff;
}

/*
 * The weight is an integer multiplied by 256.
 */
static u32 get_cluster_weight(enum cluster_type cluster)
{
	u32 weight = 0;

	switch(cluster) {
	case CLUSTER_BIG:
		weight = 1 * 256;
		break;
	case CLUSTER_LITTLE:
		weight = 2 * 256;
		break;
	}

	return weight;
}

static int get_cache_static_power_coeff(enum cluster_type cluster)
{
	int coeff = 0;

	switch(cluster) {
	case CLUSTER_BIG:
		/* 64mW @ 85C/0.9V */
		coeff = 88;
		break;
	case CLUSTER_LITTLE:
		/* 53mW @ 85C/0.9V */
		coeff = 73;
		break;
	}

	return coeff;
}

static unsigned long get_temperature_scale(unsigned long temp)
{
	int i, t_exp = 1, t_scale = 0;
	int coeff[] = { 32000, 4700, -80, 2 }; // * 1E6

	for (i = 0; i < 4; i++) {
		t_scale += coeff[i] * t_exp;
		t_exp *= temp;
	}

	return t_scale / 1000; // the value returned needs to be /1E3
}

static unsigned long get_voltage_scale(unsigned long u_volt)
{
	unsigned long m_volt = u_volt / 1000;
	unsigned long v_scale;

	v_scale = m_volt * m_volt * m_volt; // = (m_V^3) / (900 ^ 3) =

	return v_scale / 1000000; // the value returned needs to be /(1E3)
}

/* voltage in uV and temperature in mC */
static u32 get_static_power(cpumask_t *cpumask, unsigned long u_volt,
			    unsigned long milli_temp)
{
	unsigned long t_scale, v_scale;
	u32 cpu_coeff, mw_leakage;
	int nr_cpus = cpumask_weight(cpumask);
	enum cluster_type cluster =
		topology_physical_package_id(cpumask_any(cpumask));

	cpu_coeff = get_cpu_static_power_coeff(cluster);

	t_scale = get_temperature_scale(milli_temp / 1000);
	v_scale = get_voltage_scale(u_volt);

	mw_leakage = nr_cpus * (cpu_coeff * t_scale * v_scale) / 1000000;

	if (nr_cpus) {
		u32 cache_coeff = get_cache_static_power_coeff(cluster);
		mw_leakage += (cache_coeff * v_scale * t_scale) / 1000000; /* cache leakage */
	}

	return mw_leakage;
}

static int get_temp_value(void *data, long *temp)
{
	struct scpi_sensor *sensor = (struct scpi_sensor *)data;
	u32 val;
	int ret;

	ret = scpi_get_sensor_value(sensor->sensor_id, &val);

	if (!ret)
		*temp = (unsigned long)val;

	return ret;
}

static int scpi_thermal_probe(struct platform_device *pdev)
{
	struct scpi_sensor *sensor_data = &scpi_temp_sensor;
	struct device_node *np;
	int sensor, cpu;
	int i;

	if (!cpufreq_frequency_get_table(0)) {
		dev_info(&pdev->dev,
			"Frequency table not initialized. Deferring probe...\n");
		return -EPROBE_DEFER;
	}

	platform_set_drvdata(pdev, sensor_data);

	for_each_possible_cpu(cpu) {
		int cluster_id = topology_physical_package_id(cpu);
		if (cluster_id > NUM_CLUSTERS) {
			pr_warn("Cluster id: %d > %d\n", cluster_id, NUM_CLUSTERS);
			goto error;
		}

		cpumask_set_cpu(cpu, &sensor_data->cluster[cluster_id]);
	}

	for (i = 0; i < NUM_CLUSTERS; i++) {
		char node[16];
		struct cpumask *mask = &sensor_data->cluster[i];
		enum cluster_type cluster =
			topology_physical_package_id(cpumask_any(&sensor_data->cluster[i]));

		snprintf(node, 16, "cluster%d", i);
		np = of_find_node_by_name(NULL, node);

		if (!np)
			dev_info(&pdev->dev, "Node not found: %s\n", node);

		sensor_data->pactor[i] =
			power_cpu_actor_register(np, cpumask_any(mask),
						get_dyn_power_coeff(cluster),
						get_cluster_weight(cluster),
						get_static_power);

		if (IS_ERR(sensor_data->pactor[i]))
			dev_warn(&pdev->dev,
				"Error registering cpu power actor: %d\n", i);
	}

	if ((sensor = scpi_get_sensor(SOC_SENSOR)) < 0) {
		dev_warn(&pdev->dev, "%s not found. ret=%d\n", SOC_SENSOR, sensor);
		goto error;
	}

	sensor_data->sensor_id = (u16)sensor;
	dev_info(&pdev->dev, "Probed %s sensor. Id=%hu\n", SOC_SENSOR, sensor_data->sensor_id);

	sensor_data->tzd = thermal_zone_of_sensor_register(&pdev->dev,
							sensor_data->sensor_id,
							sensor_data,
							get_temp_value, NULL);

	if (IS_ERR(sensor_data->tzd)) {
		dev_warn(&pdev->dev, "Error registering sensor: %p\n", sensor_data->tzd);
		return PTR_ERR(sensor_data->tzd);
	}

	thermal_zone_device_update(sensor_data->tzd);

	return 0;

error:
	return -ENODEV;
}

static int scpi_thermal_remove(struct platform_device *pdev)
{
	struct scpi_sensor *sensor = platform_get_drvdata(pdev);

	thermal_zone_device_unregister(sensor->tzd);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct of_device_id scpi_thermal_of_match[] = {
	{ .compatible = "arm,scpi-thermal" },
	{},
};
MODULE_DEVICE_TABLE(of, scpi_thermal_of_match);

static struct platform_driver scpi_thermal_platdrv = {
	.driver = {
		.name		= "scpi-thermal",
		.owner		= THIS_MODULE,
		.of_match_table = scpi_thermal_of_match,
	},
	.probe	= scpi_thermal_probe,
	.remove	= scpi_thermal_remove,
};
module_platform_driver(scpi_thermal_platdrv);

MODULE_LICENSE("GPL");
