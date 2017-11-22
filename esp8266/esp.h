#ifndef ESP_H
#define ESP_H

#include "esp_queue.h"

enum esp_ap_mode
{
	esp_ap_mode_station = 1,
	esp_ap_mode_ap = 2,
	esp_ap_mode_dual = 3
};

enum esp_softap_mode
{
	esp_softap_mode_open = 0,
	esp_softap_mode_wpa_psk = 2,
	esp_softap_mode_wpa2_psk = 3,
	esp_softap_mode_wpa_wpa2_psk = 4
};

#define ESP_STREAM "TCP"
#define ESP_DGRAM "UDP"

typedef struct esp esp_t;

typedef int (*esp_getc_t)(esp_t *ctx);
typedef int (*esp_write_t)(esp_t *ctx, char *buf, int len);

struct token
{
	char token[16];
	char *position;
	char *end;
	int retval;
};

enum evt_type
{
	msg,
	disconnect
};

struct cxn_evt
{
	int cxn_id;
};

struct cxn_read_evt
{
	enum evt_type type;
	int len;
	char msg[2048];
};

/**
 * @brief	ESP context object.
 */
struct esp
{
	struct token tok[16];
	int tok_count;

	struct token ipd_tok;

	struct token cx_tok;
	int cx_mode;
	struct token dcx_tok;
	int dcx_mode;

	int cx_id;
	int ipd_mode;

	char msg_len_s[10];
	char *msg_len_ptr;
	char *msg_ptr;

	// A queue filled with messages and disconnect tokens
	esp_queue_t cxn_evt[5];
	struct cxn_read_evt *evt;

	// A queue for storing the connection messages
	esp_queue_t cxn;

	// Linked list of connection events (not disconnect events)
	struct cxn_event *cxn_events;

	// Callbacks for writing and getting chars
	esp_getc_t esp_getc;
	esp_write_t esp_write;

};

/**
 * @brief	Initialises ESP context.
 */
void esp_create(esp_t *ctx, esp_getc_t getc, esp_write_t write);

void esp_register_token(esp_t *ctx, char *tok, int retval);

void esp_clear_tokens(esp_t *ctx);

int esp_process(esp_t *ctx);

/* ****** Basic commands *******/
/**
 * @brief	Reset the ESP.
 */
int esp_reset(esp_t *ctx);

/**
 * @brief	Gets the ESP firmware version.
 */
int esp_version(esp_t *ctx);

/* ****** WIFI commands *******/
/**
 * @brief	Switch between station and AP mode (or both). Only station mode is currently supported.
 */
int esp_ap_mode(esp_t *ctx, enum esp_ap_mode mode);

/**
 * @brief	Connect to an AP. cwjap
 */
int esp_ap_join(esp_t *ctx, char *ssid, char *pwd);

/**
 * @brief	Lists available APs. cwlap
 */
int esp_ap_list(esp_t *ctx);

/**
 * @brief	Disconnect from an AP.
 */
int esp_ap_disconnect(esp_t *ctx);

/**
 * @brief	Configures soft AP mode.
 */
int esp_ap_configure(esp_t *ctx, char *ssid, char *pwd, int channel, enum esp_softap_mode ecn);

/* ****** TCP/IP commands *******/
/**
 * @brief	Enable/disable muxing.
 */
int esp_cipmux(esp_t *ctx, int enable);

/**
 * @brief	Bind to a port (start a server).
 *
 * AFAIK this starts both a TCP and a UDP server so there is no need to specify.
 * OK so it looks like UDP doesnt work here... maybe a UDP server isn't possible.
 */
int esp_bind(esp_t *ctx, int port);

/**
 * @brief	Unbind? Sort of like closing the socket (execpt there is no socket). (Basically deletes a server).
 */
int esp_unbind(esp_t *ctx);

/**
 * @brief	Accept an incoming connection.
 */
int esp_accept(esp_t *ctx);

/**
 * @brief	Connect to a remote server.
 */
int esp_connect(esp_t *ctx, int id, char *type, char *addr, int port);

/**
 * @brief	Read from a TCP connection.
 */
int esp_read(esp_t *ctx, int id, char *buf, int len);

#if 0

/**
 * @brief	Receive 1 packet from a UDP connection.
 */
int esp_recv(esp_t *ctx, int id, char *buf, int len);
#endif
/**
 * @brief	Write to a TCP connection.
 */
int esp_write(esp_t *ctx, int id, char *buf, int len);

/**
 * @brief	Send 1 packet to a UDP connection.
 */
int esp_send(esp_t *ctx, int id, char *buf, int len);

/**
 * @brief	Close a UDP or TCP connection. Use id=5 to close all connections.
 */
int esp_close(esp_t *ctx, int id);

#endif
