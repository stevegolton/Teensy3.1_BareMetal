#include <stdlib.h>
#include "esp_queue.h"

extern void *esp_malloc(int size);
extern void esp_free(void *ptr);

/**
 * Creates an instance.
 */
void esp_queue_create(esp_queue_t *esp_q)
{
	esp_q->next = 0;
}

/**
 * @brief	Allocates an element but doesn't commit it yet...
 */
void *esp_queue_alloc(int size)
{
	struct element *el = (struct element *)esp_malloc(size + sizeof(void *));
	return (void*)(&el->data);
}

/**
 * @brief	Commits a previously allocated element.
 */
void esp_queue_commit(esp_queue_t *esp_q, void *ptr)
{
	struct element *el = esp_q->next;

	// Roll back to get the pointer
	struct element *new_el = (struct element *)(((char *)ptr) - sizeof(void *));
	new_el->next = 0;

	// Find the end of the linked list
	while(el && el->next)
	{
		el = el->next;
	}

	// Fix the next pointer to this element
	if(el)
	{
		el->next = new_el;
	}
	else
	{
		esp_q->next = new_el;
	}
}

/**
 * @brief	Gets an element from the queue but doesn't free it.
 */
void *esp_queue_dequeue(esp_queue_t *esp_q)
{
	// Always just take the first one
	struct element *el = esp_q->next;

	if(!el) return 0;

	esp_q->next = el->next;

	// Returning the event, please free me after you're done with me
	return &el->data;
}

/**
 * @brief	Frees an element.
 */
void esp_queue_free(void *ptr)
{
	// Wind back the pointer and free it
	void *p = ((char*)ptr) - sizeof(void*);
	esp_free(p);
}

/**
 * @brief	Counts the number of elements in the queue.
 */
int esp_queue_len(esp_queue_t *esp_q)
{
	int count = 0;

	struct element *evt = esp_q->next;

	// Try to find the end of the linked list
	while(evt)
	{
		evt = evt->next;
		count ++;
	}

	return count;
}
