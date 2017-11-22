#ifndef ESP_QUEUE_H
#define ESP_QUEUE_H

struct element
{
	struct element *next;
	int data;
};

typedef struct
{
	struct element *next;

} esp_queue_t;

/**
 * Creates an instance.
 */
void esp_queue_create(esp_queue_t *esp_q);

/**
 * @brief	Allocates an element but doesn't commit it yet...
 */
void *esp_queue_alloc(int size);

/**
 * @brief	Commits a previously allocated element.
 */
void esp_queue_commit(esp_queue_t *esp_q, void *ptr);

/**
 * @brief	Gets an element from the queue but doesn't free it.
 */
void *esp_queue_dequeue(esp_queue_t *esp_q);

/**
 * @brief	Frees an element.
 */
void esp_queue_free(void *ptr);

/**
 * @brief	Counts the number of elements in the queue.
 */
int esp_queue_len(esp_queue_t *esp_q);

#endif
