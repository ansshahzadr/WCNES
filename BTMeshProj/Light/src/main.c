#include <sys/printk.h>
#include <drivers/gpio.h>
#include <devicetree.h>
#include <stdlib.h>
#include <device.h>
#include <drivers/hwinfo.h>
#include <bluetooth/bluetooth.h>
#include <settings/settings.h>
#include <bluetooth/mesh.h>

// GPIO for the Thingy LED controller
const struct device *led_ctrlr;

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define PORT	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN		DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)


// states and state changes
uint8_t onoff_state;

bool publish = false;
uint16_t reply_addr;
uint8_t reply_net_idx;
uint8_t reply_app_idx;
struct bt_mesh_model *reply_model;

// usually set by the manufacturer - hard coded here for convenience

//static const uint8_t dev_uuid[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x01 };

// device UUID
static uint8_t dev_uuid[16];

void led_on(void)
{
	// LEDs on Thingy are "active low" so zero means on. Args are expressed as RGB 0-255 values so we map them to GPIO low/high.
	gpio_pin_set(led_ctrlr, PIN, 1);
}

void led_off(void)
{
	gpio_pin_set(led_ctrlr, PIN, 0);
}

// provisioning callback functions
static void attention_on(struct bt_mesh_model *model)
{
	printk("attention_on()\n");
	led_on();
}

static void attention_off(struct bt_mesh_model *model)
{
	printk("attention_off()\n");
	led_off();
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static int provisioning_output_pin(bt_mesh_output_action_t action, uint32_t number) {
	printk("OOB Number: %u\n", number);
	return 0;
}

static void provisioning_complete(uint16_t net_idx, uint16_t addr) {
    printk("Provisioning completed\n");
}

static void provisioning_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

// provisioning properties and capabilities
static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 4,
	.output_actions = BT_MESH_DISPLAY_NUMBER,
	.output_number = provisioning_output_pin,
	.complete = provisioning_complete,
	.reset = provisioning_reset,
};

// generic onoff server message opcodes

#define BT_MESH_MODEL_OP_GENERIC_ONOFF_GET 				BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET 				BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK 		BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS 			BT_MESH_MODEL_OP_2(0x82, 0x04)


// generic onoff server functions
// need to forward declare as we have circular dependencies

void generic_onoff_status(bool publish, uint8_t on_or_off);
static void generic_onoff_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf){
	printk("gen_onoff_get\n");
	// logged for interest only
	printk("ctx net_idx=0x%02x\n",ctx->net_idx);
	printk("ctx app_idx=0x%02x\n",ctx->app_idx);
	printk("ctx addr=0x%02x\n",ctx->addr);
	printk("ctx recv_dst=0x%02x\n",ctx->recv_dst);
	reply_addr = ctx->addr;
	reply_net_idx = ctx->net_idx;
	reply_app_idx = ctx->app_idx;
	generic_onoff_status(true, onoff_state);
}

static void set_onoff_state(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf, bool ack)
{
	uint8_t msg_onoff_state = net_buf_simple_pull_u8(buf);
	if (msg_onoff_state == onoff_state) {
		// no state change so nothing to do
		return;
	}
	onoff_state = msg_onoff_state;
	uint8_t tid = net_buf_simple_pull_u8(buf);
	printk("set onoff state: onoff=%u TID=%u\n", onoff_state, tid);
	if (onoff_state == 0)
	{
		led_off();
	}
	else
	{
		led_on();
	}
	/*
	* 3.7.7.2 Acknowledged Set
	*/
	if (ack) {
		generic_onoff_status(false, onoff_state);
	}
	/*
	* If a server has a publish address, it is required to publish status on a state change
	* See Mesh Profile Specification 3.7.6.1.2
	*/
	if (model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		generic_onoff_status(true, onoff_state);
	}
}
static void generic_onoff_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("gen_onoff_set\n");
	set_onoff_state(model, ctx, buf, true);
}
static void generic_onoff_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("generic_onoff_set_unack\n");
	set_onoff_state(model, ctx, buf, false);
}


// generic onoff server model publication context
BT_MESH_MODEL_PUB_DEFINE(generic_onoff_pub, NULL, 3);
static const struct bt_mesh_model_op generic_onoff_op[] = {
	{BT_MESH_MODEL_OP_GENERIC_ONOFF_GET, 0, generic_onoff_get},
	{BT_MESH_MODEL_OP_GENERIC_ONOFF_SET, 2, generic_onoff_set},
	{BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK, 2, generic_onoff_set_unack},
	BT_MESH_MODEL_OP_END,
};

// Light HSL Server Model - minimal subset only - would not be deemed compliant
// -------------------------------------------------------------------------------------------------------

// light HSL server message opcodes


// light HSL server functions


// light HSL server model publication context

// -------------------------------------------------------------------------------------------------------
// Configuration Server
// --------------------
// static struct bt_mesh_cfg_srv cfg_srv = {
// 		.relay = BT_MESH_RELAY_DISABLED,
// 		.beacon = BT_MESH_BEACON_DISABLED,
// 		.frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
// 		.gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
// 		.default_ttl = 7,
// 		/* 3 transmissions with 20ms interval */
// 		.net_transmit = BT_MESH_TRANSMIT(2, 20),
// };

// -------------------------------------------------------------------------------------------------------
// Health Server
// -------------

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

// -------------------------------------------------------------------------------------------------------
// Composition
// -----------

// TODO complete the composition with the generic onoff server model and light HSL server model

static struct bt_mesh_model sig_models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, generic_onoff_op,	&generic_onoff_pub, NULL),
};

// node contains elements.note that BT_MESH_MODEL_NONE means "none of this type" ands here means "no vendor models"
static struct bt_mesh_elem elements[] = {
		BT_MESH_ELEM(0, sig_models, BT_MESH_MODEL_NONE),
};

// node
static const struct bt_mesh_comp comp = {
		.cid = BT_COMP_ID_LF,
		.elem = elements,
		.elem_count = ARRAY_SIZE(elements),
};

// ----------------------------------------------------------------------------------------------------
// generic onoff status TX message producer

void generic_onoff_status(bool publish, uint8_t on_or_off)
{
	int err;
	struct bt_mesh_model *model = &sig_models[2];
	if (publish && model->pub->addr == BT_MESH_ADDR_UNASSIGNED) {
		printk("No publish address associated with the generic on off server model - add one with a configuration app like nRF Mesh\n");
		return;
	}
	if (publish) {
		struct net_buf_simple *msg = model->pub->msg;
		net_buf_simple_reset(msg);
		bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS);
		net_buf_simple_add_u8(msg, on_or_off);
		printk("publishing on off status message\n");
		err = bt_mesh_model_publish(model);
		if (err) {
			printk("bt_mesh_model_publish err %d\n", err);
		}
	} 
	else {
		uint8_t buflen = 7;
		NET_BUF_SIMPLE_DEFINE(msg, buflen);
		bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS);
		net_buf_simple_add_u8(&msg, on_or_off);
		struct bt_mesh_msg_ctx ctx = {
			.net_idx = reply_net_idx,
			.app_idx = reply_app_idx,
			.addr = reply_addr,
			.send_ttl = BT_MESH_TTL_DEFAULT,
		};
	printk("sending on off status message\n");
		
		if (bt_mesh_model_send(model, &ctx, &msg, NULL, NULL))
		{
			printk("Unable to send generic onoff status message\n");
		}
	}
}


static void bt_ready(int err)
{
	if (err)
	{
		printk("bt_enable init failed with err %d\n", err);
		return;
	}
    printk("Bluetooth initialised OK\n");
	err = bt_mesh_init(&prov, &comp);

	if (err)
	{
		printk("bt_mesh_init failed with err %d\n", err);
		return;
	}

	printk("Mesh initialised OK\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	    printk("Settings loaded\n");
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
	/* run nrfjprog -e against your board (assuming it's a Nordic board) to clear provisioning data and start again */

    if (!bt_mesh_is_provisioned()) {
    	printk("Node has not been provisioned - beaconing\n");
		bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
	} else {
    	printk("Node has already been provisioned\n");
	}

}

static void configure_led()
{
	led_ctrlr = device_get_binding(PORT);
	gpio_pin_configure(led_ctrlr, PIN, GPIO_OUTPUT);
}

void indicate_on() {
	led_on();
    k_msleep(1000);
    led_off();	
}

void main(void)
{
	printk("Light Server Node\n");
	int err = 0;
	if (IS_ENABLED(CONFIG_HWINFO)) {
		err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
	}
	if (err < 0) {
		dev_uuid[0] = 0xdd;
		dev_uuid[1] = 0xdd;
	}
	configure_led();
    indicate_on();
	err = bt_enable(bt_ready);
	if (err)
	{
		printk("bt_enable failed with err %d\n", err);
	}
}
