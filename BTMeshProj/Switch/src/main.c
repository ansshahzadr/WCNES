#include <sys/printk.h>
#include <drivers/gpio.h>
#include <devicetree.h>
#include <device.h>
#include <drivers/hwinfo.h>
#include <sys/byteorder.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <settings/settings.h>
#include <bluetooth/mesh/proxy.h>

#define BUTTON_DEBOUNCE_DELAY_MS 			350

#define SW0_NODE					DT_ALIAS(sw0)
#define PORT1						DT_GPIO_LABEL(SW0_NODE, gpios)
#define BUTTON1						DT_GPIO_PIN(SW0_NODE, gpios)
#define SW0_GPIO_FLAGS					(GPIO_INPUT | DT_GPIO_FLAGS(SW0_NODE, gpios))

#define SW1_NODE					DT_ALIAS(sw1)
#define PORT2						DT_GPIO_LABEL(SW1_NODE, gpios)
#define BUTTON2						DT_GPIO_PIN(SW1_NODE, gpios)
#define SW1_GPIO_FLAGS					(GPIO_INPUT | DT_GPIO_FLAGS(SW1_NODE, gpios))

#define LED0_NODE 					DT_ALIAS(led0)
#define LED0						DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN						DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS						DT_GPIO_FLAGS(LED0_NODE, gpios)

#define BT_MESH_MODEL_OP_GENERIC_ONOFF_GET 		BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET 		BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK 	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS 		BT_MESH_MODEL_OP_2(0x82, 0x04)

static uint32_t btn_time[2];
static uint32_t btn_last_time[2];
static struct gpio_callback gpio_btn1_cb;
static struct gpio_callback gpio_btn2_cb;
const struct device *gpio_led_port;
static struct k_work button1_work;
static struct k_work button2_work;
static uint8_t dev_uuid[16];
static uint8_t onoff_tid;
static uint8_t onoff[] = {0, 1};

static inline void ledOn(void)
{
	gpio_pin_set(gpio_led_port, PIN, 1);
}

static inline void ledOff(void)
{
	gpio_pin_set(gpio_led_port, PIN, 0);
}

static void attention_on(struct bt_mesh_model *model)
{
	printk("attention_on()\n");

	ledOn();
}

static void attention_off(struct bt_mesh_model *model)
{
	printk("attention_off()\n");

	ledOff();
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static int provisioning_output_pin(bt_mesh_output_action_t action,
				   uint32_t number)
{
	printk("OOB Number: %04d\n", number);

	return 0;
}

static void provisioning_complete(uint16_t net_idx, uint16_t addr)
{
	printk("Provisioning completed\n");
}

static void provisioning_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 4,
	.output_actions = BT_MESH_DISPLAY_NUMBER,
	.output_number = provisioning_output_pin,
	.complete = provisioning_complete,
	.reset = provisioning_reset,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

static void generic_onoff_status(struct bt_mesh_model *model,
				 struct bt_mesh_msg_ctx *ctx,
				 struct net_buf_simple *buf)
{
	uint8_t onoff_state = net_buf_simple_pull_u8(buf);

	printk("generic_onoff_status(LEDs current state): %d\n\n", onoff_state);
}

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
	{BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS, 1, generic_onoff_status},
	BT_MESH_MODEL_OP_END,
};

BT_MESH_MODEL_PUB_DEFINE(gen_onoff_cli, NULL, 2);
static struct bt_mesh_model sig_models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
		      &gen_onoff_cli, &onoff[0])
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, sig_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

int genericOnOffGet()
{
	int err;
	struct bt_mesh_model *model = &sig_models[2];
	struct net_buf_simple *msg = model->pub->msg;
	
	if (model->pub->addr == BT_MESH_ADDR_UNASSIGNED)
		return -1;
	
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_GENERIC_ONOFF_GET);
	err = bt_mesh_model_publish(model);
	if (err) {
		printk("bt_mesh_model_publish err %d\n", err);
	}

	return err;
}

int sendGenOnOffSet(uint8_t on_or_off, uint16_t message_type)
{
	int err;
	struct bt_mesh_model *model = &sig_models[2];
	struct net_buf_simple *msg = model->pub->msg;
	
	if (model->pub->addr == BT_MESH_ADDR_UNASSIGNED) {
		printk("No publish address associated with the generic on off client model\n");
		return -1;
	}
	
	bt_mesh_model_msg_init(msg, message_type);
	net_buf_simple_add_u8(msg, on_or_off);
	net_buf_simple_add_u8(msg, onoff_tid);
	onoff_tid++;
	err = bt_mesh_model_publish(model);
	if (err) {
		printk("bt_mesh_model_publish err %d\n", err);
	}

	return err;
}

void genericOnOffSet(uint8_t on_or_off)
{
	if (sendGenOnOffSet(on_or_off, BT_MESH_MODEL_OP_GENERIC_ONOFF_SET)) {
		printk("Unable to send generic onoff set message\n");
	}
}

void genericOnOffSetUnAck(uint8_t on_or_off)
{
	if (sendGenOnOffSet
	    (on_or_off, BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK)) {
		printk("Unable to send generic onoff set unack message\n");
	}
}

void button1_work_handler(struct k_work *work)
{
	static bool onoff_offset = 1;

	genericOnOffSet(onoff[onoff_offset]);

	onoff_offset = !onoff_offset;
}

void button2_work_handler(struct k_work *work)
{
	genericOnOffGet();
}

bool debounce(int btn_inx)
{
	bool ignore = false;

	btn_time[btn_inx] = k_uptime_get_32();
	if (btn_time[btn_inx] < (btn_last_time[btn_inx] + BUTTON_DEBOUNCE_DELAY_MS)) {
		ignore = true;
	} else {
		ignore = false;
	}

	btn_last_time[btn_inx] = btn_time[btn_inx];

	return ignore;
}

void button_1_pressed(const struct device *gpiob, struct gpio_callback *cb,
		      uint32_t pins)
{
	if (!debounce(0))
		k_work_submit(&button1_work);
}

void button_2_pressed(const struct device *gpiob, struct gpio_callback *cb,
		      uint32_t pins)
{
	if (!debounce(1)) {
		k_work_submit(&button2_work);
	}
}

void config_led(void)
{
	int ret;

	gpio_led_port = device_get_binding(LED0);
	if (!gpio_led_port) {
		printk("error obtaining LED port\n");
		return;
	}
	ret = gpio_pin_configure(gpio_led_port, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret) {
		printk("error configuring LED port\n");
		return;
	}

	/* switch off the LED by default */
	ledOff();
}

void config_btns(void)
{
	const struct device *gpio_port1;
	const struct device *gpio_port2;
	int ret;

	gpio_port1 = device_get_binding(PORT1);
	if (!gpio_port1) {
		printk("error obtaining port 1\n");
		return;
	}

	gpio_port2 = device_get_binding(PORT2);
	if (!gpio_port2) {
		printk("error obtaining port 2\n");
		return;
	}

	/* setup button 1 */
	k_work_init(&button1_work, button1_work_handler);
	gpio_pin_configure(gpio_port1, BUTTON1, GPIO_INPUT | SW0_GPIO_FLAGS);
	ret = gpio_pin_interrupt_configure(gpio_port1, BUTTON1, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, PORT1, BUTTON1);
		return;
	}
	gpio_init_callback(&gpio_btn1_cb, button_1_pressed, BIT(BUTTON1));
	gpio_add_callback(gpio_port1, &gpio_btn1_cb);

	/* setup button 2 */
	k_work_init(&button2_work, button2_work_handler);
	gpio_pin_configure(gpio_port2, BUTTON2, GPIO_INPUT | SW1_GPIO_FLAGS);
	ret = gpio_pin_interrupt_configure(gpio_port2, BUTTON2, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, PORT2, BUTTON2);
		return;
	}
	gpio_init_callback(&gpio_btn2_cb, button_2_pressed, BIT(BUTTON2));
	gpio_add_callback(gpio_port2, &gpio_btn2_cb);
}

static void bt_ready(int err)
{
	if (err) {
		printk("bt_enable init failed with err %d\n", err);
		return;
	}

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("bt_mesh_init failed with err %d\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
		printk("Settings loaded\n");
	}

	if (!bt_mesh_is_provisioned()) {
		printk("Node has not been provisioned - beaconing\n");
		bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
	} else {
		printk("Node has already been provisioned\n");
	}
}

void main(void)
{
	int err = 0;

	if (IS_ENABLED(CONFIG_HWINFO)) {
		err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));	
		if (err) {
			dev_uuid[0] = 0xdd;
			dev_uuid[1] = 0xdd;
		}
	}

	onoff_tid = 0;

	config_btns();

	config_led();
	
	err = bt_enable(bt_ready);
	if (err)
		printk("bt_enable failed with err %d\n", err);
}
