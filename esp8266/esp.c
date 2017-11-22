#ifdef ESP_VERBOSE
#include <stdio.h>
#endif

#include "esp.h"
#include "esp_queue.h"

// I'm not proud of this macro, but it saves on typing.
// Assumes that "ctx" is defined somewhere in scope.
#define WRITE(x) ctx->esp_write(ctx, x, esp_strlen(x))

// Gets the number of elements in an array
#define NUMEL(x) (sizeof(x)/sizeof(x[0]))

/**
 * Prints an int as a decimal string (only works for positive integers).
 */
void esp_itoa(int num, char *buf)
{
	char *cursor;

	if(num == 0)
	{
		*buf = '0';
		buf++;
		*buf = 0;
		return;
	}

	if(num < 0)
	{
		num *= -1;
		*buf = '-';
		buf++;
	}

	cursor = buf;

	while(num > 0)
	{
		*cursor = (num % 10) + '0';
		num /= 10;
		cursor++;
	}

	// And reverse the string now
	char *end = cursor;
	cursor --;
	char tmp;
	while(cursor > buf)
	{
		tmp = *cursor;
		*cursor = *buf;
		*buf = tmp;
		cursor--;
		buf++;
	}

	*end = 0;
}

int esp_atoi(const char *buf)
{
	int negative = 1;
	if(*buf == '-')
	{
		negative = -1;
		buf++;
	}

	int num = 0;
	while(*buf)
	{
		num *= 10;
		num += (*buf) - '0';
		buf++;
	}

	return num * negative;
}

void *esp_memset(void *buf, int val, int len)
{
	int i;
	char *buf_c = (char*)buf;
	for(i = 0; i < len; i++)
	{
		buf_c[i] = (char)val;
	}

	return buf;
}

void *esp_memcpy(void *dest, const void *src, int len)
{
	int i;
	char *dest_c = (char*)dest;
	char *src_c = (char*)src;
	for(i = 0; i < len; i++)
	{
		dest_c[i] = src_c[i];
	}

	return 0; // TODO what to return here??
}

int esp_strlen(const char *str)
{
	int len = 0;
	while(*str)
	{
		str++;
		len++;
	}

	return len;
}

void *esp_strcpy(void *dest, const void *src)
{
	return esp_memcpy(dest, src, esp_strlen(src) + 1);
}

static void configure_tok(struct token *tokp, char *tok, int retval)
{
	// Fill in the token structure
	esp_strcpy(tokp->token, tok);
	tokp->position = tokp->token;
	tokp->end = tokp->position + esp_strlen(tok);
	tokp->retval = retval;
}

void esp_create(esp_t *ctx, esp_getc_t esp_getc, esp_write_t esp_write)
{
	esp_memset(ctx, 0, sizeof(*ctx));

	ctx->tok_count = 0;
	ctx->ipd_mode = 0;
	ctx->msg_len_ptr = ctx->msg_len_s;
	configure_tok(&ctx->ipd_tok, "+IPD", 1);
	configure_tok(&ctx->cx_tok, ",CONNECT\r\n", 0);
	configure_tok(&ctx->dcx_tok, ",CLOSED\r\n", 0);
	ctx->cxn_events = 0;
	ctx->cx_mode = 0;
	ctx->dcx_mode = 0;

	// Initialize the queues
	esp_queue_create(&ctx->cxn);
	int i;
	for(i = 0; i < NUMEL(ctx->cxn_evt); i++)
	{
		esp_queue_create(&ctx->cxn_evt[i]);
	}

	// Set up call-backs
	ctx->esp_getc = esp_getc;
	ctx->esp_write = esp_write;
}

void esp_register_token(esp_t *ctx, char *tok, int retval)
{
	// Fill in the token structure
	struct token *tokp = &ctx->tok[ctx->tok_count];
	configure_tok(tokp, tok, retval);
	ctx->tok_count++;
}

void esp_clear_tokens(esp_t *ctx)
{
	ctx->tok_count = 0;
}

int match_tok(struct token *tokp, char in)
{
	if(*tokp->position == in)
	{
		// This char matches, keep matching...
		tokp->position++;
	}
	else
	{
		// Found one error, get out after resetting position
		tokp->position = tokp->token;
		return 0;
	}

	if(tokp->position == tokp->end)
	{
		tokp->position = tokp->token;
		return 2;
	}


	// If we are here then we have matched everything so far...
	return 1;
}

int find_tok(struct token *tokp, char in)
{
	if(*tokp->position == in)
	{
		tokp->position++;
	}
	else
	{
		tokp->position = tokp->token;

		if(*tokp->position == in)
			tokp->position++;
	}

	if(tokp->position == tokp->end)
	{
		return 1;
	}

	return 0;
}



int esp_process(esp_t *ctx)
{
	char in = ctx->esp_getc(ctx);

#ifdef ESP_VERBOSE
	// Print every character read to stdout
	putc(in, stdout);
#endif

	int i;
	struct token *tokp;

	if(ctx->cx_mode)
	{
		// Look for CONNECT
		int retval = match_tok(&ctx->cx_tok, in);
		if(retval == 2)
		{
			// Enqueue connect token and get out
#ifdef ESP_VERBOSE
			printf("[CXN:%d]", ctx->cx_id);
#endif
			struct cxn_evt *evt = (struct cxn_evt *)esp_queue_alloc(sizeof(struct cxn_evt));
			evt->cxn_id = ctx->cx_id;
			esp_queue_commit(&ctx->cxn, (char *)evt);

			ctx->cx_mode = 0;
		}
		else if(retval == 0)
		{
			ctx->cx_mode = 0;
		}
	}

	if(ctx->dcx_mode)
	{
		// Look for DISCONNECT
		int retval = match_tok(&ctx->dcx_tok, in);
		if(retval == 2)
		{
			// Enqueue connect token and get out
#ifdef ESP_VERBOSE
			printf("[DCXN:%d]", ctx->cx_id);
#endif

			// TODO assert ctx->cx_id < NUMEL(ctx->cxn_evt)
			struct cxn_read_evt *evt = (struct cxn_read_evt *)esp_queue_alloc(sizeof(enum evt_type));
			evt->type = disconnect;
			esp_queue_commit(&ctx->cxn_evt[ctx->cx_id], (char *)evt);

			ctx->dcx_mode = 0;
		}
		else if(retval == 0)
		{
			ctx->dcx_mode = 0;
		}
	}

	// are we in the middle of an IPD?
	if(ctx->ipd_mode)
	{
		switch(ctx->ipd_mode)
		{
		case 1:
			if(in == ',') ctx->ipd_mode = 2;
			break;

		case 2:
			if(in == ',')
			{
				ctx->ipd_mode = 3;
			}
			else
			{
				// Store the connection ID away
				ctx->cx_id = in - '0';
			}
			break;

		case 3:
			// Copy length
			if(in == ':')
			{
				// TODO assert ctx->cx_id < NUMEL(ctx->cxn_evt)
				struct cxn_read_evt *evt = (struct cxn_read_evt *)esp_queue_alloc(sizeof(enum evt_type) + sizeof(int) + esp_atoi(ctx->msg_len_s));
				evt->type = msg;
				ctx->evt = evt;
				ctx->evt->len = esp_atoi(ctx->msg_len_s);

				ctx->ipd_mode = 4;
				ctx->msg_ptr = ctx->evt->msg;
				break;
			}
			*ctx->msg_len_ptr = in;
			ctx->msg_len_ptr++;
			break;

		case 4:
			// Copy msg up to msg_len
			if((ctx->msg_ptr - ctx->evt->msg) == ctx->evt->len)
			{
				esp_queue_commit(&ctx->cxn_evt[ctx->cx_id], (char *)ctx->evt);
				ctx->ipd_mode = 0;
#ifdef ESP_VERBOSE
				printf("[RX:%d,%d]", ctx->cx_id, ctx->evt->len);
#endif

				break;
			}
			*ctx->msg_ptr = in;
			ctx->msg_ptr++;
			break;

		}

		return -101;
	}


	// Try matching all queued tokens
	for(i = 0; i < ctx->tok_count; i++)
	{
		tokp = &ctx->tok[i];
		if(*tokp->position == in)
		{
			tokp->position++;
		}
		else
		{
			tokp->position = tokp->token;

			if(*tokp->position == in)
				tokp->position++;
		}

		if(tokp->position == tokp->end)
		{
#ifdef ESP_VERBOSE
			printf("[TOK:%s]", tokp->token);
#endif
			return tokp->retval;
		}
	}

	tokp = &ctx->ipd_tok;
	if(*tokp->position == in)
	{
		tokp->position++;
	}
	else
	{
		tokp->position = tokp->token;

		if(*tokp->position == in)
			tokp->position++;
	}

	if(tokp->position == tokp->end)
	{
		// IPD found, start parsing the message for later
		ctx->msg_len_ptr = ctx->msg_len_s;
		ctx->ipd_mode = 1;
	}

	if(in >= '0' && in <= '4')
	{
		// This might be the beginning of a connect or a disconnect event
		ctx->cx_mode = 1;
		ctx->dcx_mode = 1;
		ctx->cx_id = in - '0';
	}

	// No matches?
	return -100;
}

int esp_reset(esp_t *ctx)
{
	WRITE("AT+RST\r\n");

	// TODO add more tokens for error cases
	esp_register_token(ctx, "ready", 0);

	int err;
	while(1)
	{
		// TODO timeout
		err = esp_process(ctx);
		if(0 == err) break;
	}

	esp_clear_tokens(ctx);

	return err;
}

int esp_version(esp_t *ctx)
{
	// TODO not supported
	return -1;
}

int esp_ap_mode(esp_t *ctx, enum esp_ap_mode mode)
{
	char buf[32];
	esp_itoa((int)mode, buf);

	WRITE("AT+CWMODE=");
	WRITE(buf);
	WRITE("\r\n");

	// TODO add more tokens for error cases
	esp_register_token(ctx, "OK", 0);

	int err;
	while(1)
	{
		// TODO timeout
		err = esp_process(ctx);
		if(0 == err) break;
	}

	esp_clear_tokens(ctx);

	return err;
}

int esp_ap_join(esp_t *ctx, char *ssid, char *pwd)
{
	WRITE("AT+CWJAP=\"");
	WRITE(ssid);
	WRITE("\",\"");
	WRITE(pwd);
	WRITE("\"\r\n");

	esp_register_token(ctx, "OK", 0);
	esp_register_token(ctx, "ERROR", -1);

	int err;
	while(1)
	{
		// TODO timeout
		err = esp_process(ctx);
		if(err > -100) break;
	}

	esp_clear_tokens(ctx);

	return err;
}

int esp_ap_list(esp_t *ctx)
{
	// TODO not supported
	return -1;
}

int esp_ap_disconnect(esp_t *ctx)
{
	WRITE("AT+CWQAP\r\n");

	// TODO add more tokens for error cases
	esp_register_token(ctx, "OK", 0);

	int err;
	while(1)
	{
		// TODO timeout
		err = esp_process(ctx);
		if(0 == err) break;
	}

	esp_clear_tokens(ctx);

	return err;
}

int esp_ap_configure(esp_t *ctx, char *ssid, char *pwd, int channel, enum esp_softap_mode ecn)
{
	char channel_s[32];
	esp_itoa(channel, channel_s);

	char ecn_s[32];
	esp_itoa(ecn, ecn_s);

	WRITE("AT+CWSAP=\"");
	WRITE(ssid);
	WRITE("\",\"");
	WRITE(pwd);
	WRITE("\",");
	WRITE(channel_s);
	WRITE(",");
	WRITE(ecn_s);
	WRITE("\r\n");

	// TODO add more tokens for error cases
	esp_register_token(ctx, "OK", 0);
	esp_register_token(ctx, "ERROR", -1);

	int err;
	while(1)
	{
		// TODO timeout
		err = esp_process(ctx);
		if(err > -100) break;
	}

	esp_clear_tokens(ctx);

	return err;
}

int esp_read(esp_t *ctx, int id, char *buf, int len)
{
	int retval;
	struct cxn_read_evt *evt;

	// First, check for any unread events, otherwise just keep reading characters from the ESP
	while(0 == (evt = esp_queue_dequeue(&ctx->cxn_evt[id])))
	{
		// TODO timeout
		esp_process(ctx);
	}

	switch(evt->type)
	{
		case msg:
		{
			// This event was a message
			esp_memcpy(buf, evt->msg, evt->len);
			retval = evt->len;
			break;
		}
		case disconnect:
		{
			retval = -1;
			break;
		}
		default:
		{
			// I don't really know what to do here, this means that someone enqueued an event
			// type which doesn't exist... this is more of a programming error.
			retval = -2;
			break;
		}
	}

	esp_queue_free(evt);

	return retval;
}

int esp_cipmux(esp_t *ctx, int enable)
{
	char enable_s[32];
	esp_itoa(enable, enable_s);

	WRITE("AT+CIPMUX=");
	WRITE(enable_s);
	WRITE("\r\n");

	// TODO add more tokens for error cases
	esp_register_token(ctx, "OK", 0);
	esp_register_token(ctx, "no change", 0);
	esp_register_token(ctx, "we must restart", 0);
	esp_register_token(ctx, "ERROR", -1);

	int err;
	while(1)
	{
		// TODO timeout
		err = esp_process(ctx);
		if(err > -100) break;
	}

	esp_clear_tokens(ctx);

	return err;
}

int esp_bind(esp_t *ctx, int port)
{
	char port_s[32];
	esp_itoa(port, port_s);

	WRITE("AT+CIPSERVER=1,");
	WRITE(port_s);
	WRITE("\r\n");

	// TODO add more tokens for error cases
	esp_register_token(ctx, "OK", 0);

	int err;
	while(1)
	{
		// TODO timeout
		err = esp_process(ctx);
		if(0 == err) break;
	}

	esp_clear_tokens(ctx);

	return err;
}

int esp_unbind(esp_t *ctx)
{
	WRITE("AT+CIPSERVER=0\r\n");

	// TODO add more tokens for error cases
	esp_register_token(ctx, "OK", 0);

	int err;
	while(1)
	{
		// TODO timeout
		err = esp_process(ctx);
		if(0 == err) break;
	}

	esp_clear_tokens(ctx);

	return err;
}

int esp_accept(esp_t *ctx)
{
	int id;

	// First, check for any unread events, otherwise just keep reading characters from the ESP
	struct cxn_evt *evt = 0;
	while(0 == (evt = esp_queue_dequeue(&ctx->cxn)))
	{
		// TODO timeout
		esp_process(ctx);
	}

	id = evt->cxn_id;
	esp_queue_free(evt);

	return id;
}

int esp_connect(esp_t *ctx, int id, char *type, char *addr, int port)
{
	char id_s[32];
	esp_itoa(id, id_s);

	char port_s[32];
	esp_itoa(port, port_s);

	WRITE("AT+CIPSTART=");
	WRITE(id_s);
	WRITE(",\"");
	WRITE(type);
	WRITE("\",\"");
	WRITE(addr);
	WRITE("\",");
	WRITE(port_s);
	WRITE("\r\n");

	// TODO add more tokens for error cases
	esp_register_token(ctx, "OK", 0);

	int err;
	while(1)
	{
		// TODO timeout
		err = esp_process(ctx);
		if(0 == err) break;
	}

	esp_clear_tokens(ctx);

	return err;
}

int esp_write(esp_t *ctx, int id, char *bufin, int len)
{
	char id_s[32];
	esp_itoa(id, id_s);

	char len_s[32];
	esp_itoa(len, len_s);

	WRITE("AT+CIPSEND=");
	WRITE(id_s);
	WRITE(",");
	WRITE(len_s);
	WRITE("\r\n");

	// TODO add more tokens for error cases
	esp_register_token(ctx, ">", 0);
	esp_register_token(ctx, "link is not", -1);
	esp_register_token(ctx, "busy p...", -2);

	int err;
	while(1)
	{
		// TODO timeout
		err = esp_process(ctx);
		if(err > -99) break;
	}

	esp_clear_tokens(ctx);

	if(err == 0)
	{
		esp_register_token(ctx, "OK", 0);
		ctx->esp_write(ctx, bufin, len);

		while(1)
		{
			// Read 1 char
			err = esp_process(ctx);
			if(0 == err) break;
		}

		esp_clear_tokens(ctx);
	}

	return err;
}

int esp_send(esp_t *ctx, int id, char *buf, int len)
{
	// Just proxy to the TCP style "write" function
	return esp_write(ctx, id, buf, len);
}

int esp_close(esp_t *ctx, int id)
{
	char id_s[32];
	struct cxn_read_evt *evt;

	esp_itoa(id, id_s);

	WRITE("AT+CIPCLOSE=");
	WRITE(id_s);
	WRITE("\r\n");

	// TODO add more tokens for error cases
	esp_register_token(ctx, "OK", 0);
	esp_register_token(ctx, "link is not", -1);

	int err;
	while(1)
	{
		// TODO timeout
		err = esp_process(ctx);
		if(err > -99) break;
	}

	esp_clear_tokens(ctx);

	// Clear down event queue
	while(0 != (evt = esp_queue_dequeue(&ctx->cxn_evt[id])))
	{
		esp_queue_free(evt);
	}

	return err;
}
