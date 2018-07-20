#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

#define PRO_DEV_NAME "protean-pwm"
#define PRO_DEV_MAX_REG 0x0C
#define PRO_DEV_MAX_CHANNELS 6
#define PRO_DEV_REG_MODE 0x00
#define PRO_DEV_REG_CH1 0x01
#define PRO_COG_FREQ 80000000L
#define PRO_NS_SEC 1000000000L


enum protean_pwm_mode {
	PRO_MODE_MEASURE = 0,
	PRO_MODE_GEN     = 1
};


struct protean_pwm {
	struct device *dev;
	struct regmap *regmap;
	struct pwm_chip chip;
	enum protean_pwm_mode mode;
};


/**
 * Define readable and Writable registers
 */
static const struct regmap_range read_ranges[] = {
	{ 0x01, 0x06 }, // channel registers
	{ 0x0A, 0x0A }, // firmware version register
	{ 0x0C, 0x0C }, // rotory encoder register
};

static const struct regmap_range write_ranges[] = {
	{ 0x00, 0x06 }, // config + channel registers
	{ 0x0B, 0x0B }, // reset pseudo register
};

static const struct regmap_access_table read_table = {
	.yes_ranges = read_ranges,
	.n_yes_ranges = sizeof(read_ranges) / sizeof(struct regmap_range),
	.no_ranges = write_ranges,
	.n_no_ranges = sizeof(write_ranges) / sizeof(struct regmap_range),
};

static const struct regmap_access_table write_table = {
	.yes_ranges = write_ranges,
	.n_yes_ranges = sizeof(write_ranges) / sizeof(struct regmap_range),
	.no_ranges = read_ranges,
	.n_no_ranges = sizeof(read_ranges) / sizeof(struct regmap_range),
};


static inline struct protean_pwm *to_protean(struct pwm_chip *chip)
{
	return container_of(chip, struct protean_pwm, chip);
}


static int set_polarity(struct pwm_chip* chip,
                        struct pwm_device* dev,
                        enum pwm_polarity polarity)
{
	return -EINVAL;
}

static int write_channel(struct pwm_chip* chip,
                      struct pwm_device* dev,
					  int duty_ns,
					  int period_ns)
{
	struct protean_pwm *protean_pwm = to_protean(chip);

	// justification:
	// PRO_NS_SEC / PRO_COG_FREQ -> ns per clock cycle (12.5)
	// to avoid fractions
	// (10 * PRO_NS_SEC) / PRO_COG_FREQ -> 125
	// 10 * (duty_ns / denom) -> cycles
	// bit shifted into arbitrary units
	const unsigned long denom = 125;//(10L * PRO_NS_SEC) / PRO_COG_FREQ;
	unsigned long duty_arb = (10L * (duty_ns / denom)) >> 10;

	if (duty_arb > 255)
	{
		return -EINVAL;
	}

	return regmap_write(protean_pwm->regmap,
	                    PRO_DEV_REG_CH1 + dev->hwpwm,
						duty_arb);
}


static int measure_channel(struct pwm_chip* chip,
                      struct pwm_device* dev,
					  struct pwm_capture *result,
					  unsigned long timeout)
{
	struct protean_pwm *protean_pwm = to_protean(chip);

	// justification:
	// PRO_NS_SEC / PRO_COG_FREQ -> ns per clock cycle (12.5)
	// to avoid fractions
	// (10 * PRO_NS_SEC) / PRO_COG_FREQ -> 125
	// 10 * (duty_ns / denom) -> cycles
	// bit shifted into arbitrary units
	const unsigned long denom = 125;//(10 * PRO_NS_SEC) / PRO_COG_FREQ;
	int duty_arb;

	regmap_read(protean_pwm->regmap,
	            PRO_DEV_REG_CH1 + dev->hwpwm,
	            &duty_arb);

	result->duty_cycle = ((duty_arb << 10) * denom) / 10;
	result->period = PRO_NS_SEC / 50; // 20 ms, standard period for rc-servos

	return 0;
}


int enable_pwm_gen(struct pwm_chip *chip, struct pwm_device *pwm)
{
		struct protean_pwm *protean_pwm = to_protean(chip);
		return regmap_write(protean_pwm->regmap, PRO_DEV_REG_MODE, 1);
}


void disable_pwm_gen(struct pwm_chip *chip, struct pwm_device *pwm)
{
		struct protean_pwm *protean_pwm = to_protean(chip);
		regmap_write(protean_pwm->regmap, PRO_DEV_REG_MODE, 1);
}


/**
 * General I2C device configuration
 */
static const struct regmap_config config = {
	.reg_bits       = 8,
	.reg_stride     = 1,
	.val_bits       = 8,
	// .fast_io        = 1, // TODO test out
	.max_register   = PRO_DEV_MAX_REG,
	.wr_table       = &write_table,
	.rd_table       = &read_table,

	.cache_type   = REGCACHE_NONE,
};

static const struct pwm_ops protean_pwm_ops = {
	.config  = write_channel,
	.capture = measure_channel,
	.enable  = enable_pwm_gen,
	.disable = disable_pwm_gen,
	.set_polarity = set_polarity,
	.owner   = THIS_MODULE,
};


static int probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct protean_pwm *protean_pwm;
	struct device *dev = &cl->dev;


	protean_pwm = devm_kzalloc(dev, sizeof(*protean_pwm), GFP_KERNEL);
	if (protean_pwm == NULL)
	{
		return -ENOMEM;
	}

	protean_pwm->regmap = devm_regmap_init_i2c(cl, &config);
	if (IS_ERR(protean_pwm->regmap))
	{
		return PTR_ERR(protean_pwm->regmap);
	}

	protean_pwm->dev = dev;
	i2c_set_clientdata(cl, protean_pwm);


	protean_pwm->chip.ops = &protean_pwm_ops;
	protean_pwm->chip.npwm = PRO_DEV_MAX_CHANNELS;
	protean_pwm->chip.base = PRO_DEV_MAX_CHANNELS;
	protean_pwm->chip.dev = dev;

	return pwmchip_add(&protean_pwm->chip);
}


static int remove(struct i2c_client *cl)
{
	struct protean_pwm *protean_pwm = i2c_get_clientdata(cl);
	
	// set it back to echo mode
	disable_pwm_gen(&protean_pwm->chip, NULL);

	return pwmchip_remove(&protean_pwm->chip);
}

static const struct i2c_device_id protean_pwm_ids[] = {
	{ PRO_DEV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, protean_pwm_ids);

static const struct of_device_id protean_pwm_of_match[] = {
	{ .compatible = "protean," PRO_DEV_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, protean_pwm_of_match);

static struct i2c_driver protean_pwm_driver = {
	.probe = probe, // TODO
	.remove = remove,
	.driver = {
		.name = PRO_DEV_NAME,
		.of_match_table = of_match_ptr(protean_pwm_of_match),
	},
};

module_i2c_driver(protean_pwm_driver);

MODULE_DESCRIPTION("Protean PWM-Logger driver");
MODULE_AUTHOR("Kirk Roerig");
MODULE_LICENSE("GPL");

#undef PRO_DEV_NAME
