/*
 * QB50_mem.c
 *
 *  Created on: 2016年4月29日
 *      Author: Administrator
 */

#include "cubesat.h"

#include "bsp_reset.h"

/* A few bytes might be lost to byte aligning the heap start address. */
#define configADJUSTED_QB50_HEAP_SIZE	( configTOTAL_QB50_HEAP_SIZE - portBYTE_ALIGNMENT_QB50 )

static uint8_t qb50nobuff = 0;

/*
 * Initialises the heap structures before their first use.
 */
static void qb50HeapInit( void );

/* Allocate the memory for the heap. */
static uint8_t qb50Heap[ configTOTAL_QB50_HEAP_SIZE ] __attribute__((section(".hk")));

/* Define the linked list structure.  This is used to link free blocks in order
of their size. */
typedef struct BLOCK_LINK
{
	struct BLOCK_LINK *pxNextFreeBlock;	/*<< The next free block in the list. */
	size_t xBlockSize;						/*<< The size of the free block. */
} Blocklink_t;

#define qb50TRUE		1
#define qb50FALSE		0

static const uint16_t qb50heapSTRUCT_SIZE	= ( ( sizeof ( Blocklink_t ) + ( portBYTE_ALIGNMENT_QB50 - 1 ) ) & ~portBYTE_ALIGNMENT_QB50_MASK );
#define qb50heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( qb50heapSTRUCT_SIZE * 2 ) )

/* Create a couple of list links to mark the start and end of the list. */
static Blocklink_t qb50Start, qb50End;

/* Keeps track of the number of free bytes remaining, but says nothing about fragmentation. */
static size_t qb50FreeBytesRemaining = configADJUSTED_QB50_HEAP_SIZE;


/*
 * Insert a block into the list of free blocks - which is ordered by size of
 * the block.  Small blocks at the start of the list and large blocks at the end
 * of the list.
 */
#define qb50InsertBlockIntoFreeList( pxBlockToInsert )								\
{																					\
	Blocklink_t *pxIterator;														\
	size_t xBlockSize;															    \
																					\
	xBlockSize = pxBlockToInsert->xBlockSize;										\
																					\
	/* Iterate through the list until a block is found that has a larger size */	\
	/* than the block we are inserting. */											\
	for( pxIterator = &qb50Start; pxIterator->pxNextFreeBlock->xBlockSize < xBlockSize; pxIterator = pxIterator->pxNextFreeBlock )	\
	{																				\
		/* There is nothing to do here - just iterate to the correct position. */	\
	}																				\
																					\
	/* Update the list to include the block being inserted in the correct */		\
	/* position. */																	\
	pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;					\
	pxIterator->pxNextFreeBlock = pxBlockToInsert;									\
}
/*-----------------------------------------------------------*/

void *qb50Malloc( size_t xWantedSize )
{
	Blocklink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
	static long int qb50HeapHasBeenInitialised = qb50FALSE;
	void *pvReturn = NULL;

	vTaskSuspendAll();
	{
		/* If this is the first call to malloc then the heap will require
		initialisation to setup the list of free blocks. */
		if( qb50HeapHasBeenInitialised == qb50FALSE )
		{
			qb50HeapInit();
			qb50HeapHasBeenInitialised = qb50TRUE;
		}

		/* The wanted size is increased so it can contain a Blocklink_t
		structure in addition to the requested amount of bytes. */
		if( xWantedSize > 0 )
		{
			xWantedSize += qb50heapSTRUCT_SIZE;

			/* Ensure that blocks are always aligned to the required number of bytes. */
			if( ( xWantedSize & portBYTE_ALIGNMENT_QB50_MASK ) != 0 )
			{
				/* Byte alignment required. */
				xWantedSize += ( portBYTE_ALIGNMENT_QB50 - ( xWantedSize & portBYTE_ALIGNMENT_QB50_MASK ) );
			}
		}

		if( ( xWantedSize > 0 ) && ( xWantedSize < configADJUSTED_QB50_HEAP_SIZE ) )
		{
			/* Blocks are stored in byte order - traverse the list from the start
			(smallest) block until one of adequate size is found. */
			pxPreviousBlock = &qb50Start;
			pxBlock = qb50Start.pxNextFreeBlock;
			while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
			{
				pxPreviousBlock = pxBlock;
				pxBlock = pxBlock->pxNextFreeBlock;
			}

			/* If we found the end marker then a block of adequate size was not found. */
			if( pxBlock != &qb50End )
			{
				/* Return the memory space - jumping over the Blocklink_t structure
				at its start. */
				pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + qb50heapSTRUCT_SIZE );

				/* This block is being returned for use so must be taken out of the
				list of free blocks. */
				pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

				/* If the block is larger than required it can be split into two. */
				if( ( pxBlock->xBlockSize - xWantedSize ) > qb50heapMINIMUM_BLOCK_SIZE )
				{
					/* This block is to be split into two.  Create a new block
					following the number of bytes requested. The void cast is
					used to prevent byte alignment warnings from the compiler. */
					pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );

					/* Calculate the sizes of two blocks split from the single
					block. */
					pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
					pxBlock->xBlockSize = xWantedSize;

					/* Insert the new block into the list of free blocks. */
					qb50InsertBlockIntoFreeList( ( pxNewBlockLink ) );
				}

				qb50FreeBytesRemaining -= pxBlock->xBlockSize;
			}else {
				if(qb50nobuff++ == 10) {
					cpu_reset();
				}
			}
		}
	}
	xTaskResumeAll();

	#if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if( pvReturn == NULL )
		{
			extern void vApplicationMallocFailedHook( void );
			vApplicationMallocFailedHook();
		}
	}
	#endif

	return pvReturn;
}
/*-----------------------------------------------------------*/

void qb50Free( void *pv )
{
uint8_t *puc = ( uint8_t * ) pv;
Blocklink_t *pxLink;

	if( pv != NULL )
	{
		/* The memory being freed will have an Blocklink_t structure immediately
		before it. */
		puc -= qb50heapSTRUCT_SIZE;

		/* This unexpected casting is to keep some compilers from issuing
		byte alignment warnings. */
		pxLink = ( void * ) puc;

		vTaskSuspendAll();
		{
			/* Add this block to the list of free blocks. */
			qb50InsertBlockIntoFreeList( ( ( Blocklink_t * ) pxLink ) );
			qb50FreeBytesRemaining += pxLink->xBlockSize;
		}
		xTaskResumeAll();
	}
}
/*-----------------------------------------------------------*/

size_t qb50GetFreeHeapSize( void )
{
	return qb50FreeBytesRemaining;
}
/*-----------------------------------------------------------*/

static void qb50HeapInit( void )
{
	Blocklink_t *pxFirstFreeBlock;
	uint8_t *pucAlignedHeap;

	/* Ensure the heap starts on a correctly aligned boundary. */
	pucAlignedHeap = ( uint8_t * ) ( ( ( portPOINTER_SIZE_TYPE_QB50 ) &qb50Heap[ portBYTE_ALIGNMENT_QB50 ] ) & ( ( portPOINTER_SIZE_TYPE_QB50 ) ~portBYTE_ALIGNMENT_QB50_MASK ) );

	/* xStart is used to hold a pointer to the first item in the list of free
	blocks.  The void cast is used to prevent compiler warnings. */
	qb50Start.pxNextFreeBlock = ( void * ) pucAlignedHeap;
	qb50Start.xBlockSize = ( size_t ) 0;

	/* xEnd is used to mark the end of the list of free blocks. */
	qb50End.xBlockSize = configADJUSTED_QB50_HEAP_SIZE;
	qb50End.pxNextFreeBlock = NULL;

	/* To start with there is a single free block that is sized to take up the
	entire heap space. */
	pxFirstFreeBlock = ( void * ) pucAlignedHeap;
	pxFirstFreeBlock->xBlockSize = configADJUSTED_QB50_HEAP_SIZE;
	pxFirstFreeBlock->pxNextFreeBlock = &qb50End;
}

