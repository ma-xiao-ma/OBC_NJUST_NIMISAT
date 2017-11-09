/*
 * obc_mem.c
 *
 *  Created on: 2017年11月9日
 *      Author: Ma Wenli
 */

#include "FreeRTOS.h"
#include "task.h"

#include "obc_mem.h"


/* 内存分割阈值 */
#define heapMINIMUM_BLOCK_SIZE  ( ( size_t ) ( heapSTRUCT_SIZE * 2 ) )

/* Assumes 8bit bytes! */
#define heapBITS_PER_BYTE       ( ( size_t ) 8 )

/* 地址对齐处理之后会丢弃几个字节 */
#define heapADJUSTED_HEAP_SIZE  ( configTOTAL_HEAP_SIZE - portBYTE_ALIGNMENT )

/* 为堆分配静态内存，定位到外部SRAM中，bss段每次复位后都会清零 */
static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] __attribute__((section(".bss.hk")));

/* 管理空闲内存块的链表结构体 */
typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK *pxNextFreeBlock;   /*<< 指向下一个空闲块 */
    size_t xBlockSize;                      /*<< 块大小（包含结构体字节开销） */
} BlockLink_t;

/*-----------------------------------------------------------*/

/**
 * 把被释放的内存块插入到链表的合适位置，被释放的内存块将会
 * 与链表中与之相邻的内存块融合成一个新的内存块
 *
 * @param pxBlockToInsert 被释放的内存块指针
 */
static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert );

/**
 * 第一次调用ObcMemMalloc()时，会自动调用堆初始化函数
 */
static void prvHeapInit( void );

/*-----------------------------------------------------------*/

/* 被放在每个申请的内存块的起始处的结构体大小，需要字节对齐 */
static const uint16_t heapSTRUCT_SIZE   = ( ( sizeof ( BlockLink_t ) + ( portBYTE_ALIGNMENT - 1 ) ) & ~portBYTE_ALIGNMENT_MASK );

/* 确保 pxEnd 指针能够 在地接对齐边界上结束 */
static const size_t xTotalHeapSize = ( ( size_t ) heapADJUSTED_HEAP_SIZE ) & ( ( size_t ) ~portBYTE_ALIGNMENT_MASK );

/* 创建两个链表元素，用以标记链表的开始和结束 */
static BlockLink_t xStart, *pxEnd = NULL;

/* xFreeBytesRemaining 变量用来保存内存堆剩余内存大小 */
static size_t xFreeBytesRemaining = ( ( size_t ) heapADJUSTED_HEAP_SIZE ) & ( ( size_t ) ~portBYTE_ALIGNMENT_MASK );
/* xMinimumEverFreeBytesRemaining 变量记录最小的空闲内存块大小 */
static size_t xMinimumEverFreeBytesRemaining = ( ( size_t ) heapADJUSTED_HEAP_SIZE ) & ( ( size_t ) ~portBYTE_ALIGNMENT_MASK );

/* size_t 类型的最高位.  当BlockLink_t结构体的xBlockSize元素的最高位为1时，
表示此内存块已经被使用，属于应用程序；当此位为0时，表示该内存块未被使用，属于空闲堆*/
static size_t xBlockAllocatedBit = 0;

/*-----------------------------------------------------------*/

/**
 * 内存申请函数
 *
 * @param xWantedSize 申请内存大小       返回空指针则说明申请内存失败
 */
void *ObcMemMalloc( size_t xWantedSize )
{
BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
void *pvReturn = NULL;

    vTaskSuspendAll();
    {
        /* 如果这是第一次调用malloc，那么堆将需要初始化来设置空闲块的链表 */
        if( pxEnd == NULL )
        {
            prvHeapInit();
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        /* 检查请求的块的大小是不是太大，最高位位设置。
         * BlockLink_t结构的块大小成员的最高位用于
         * 确定谁拥有该块--应用程序或内核，因此它必须是0。 */
        if( ( xWantedSize & xBlockAllocatedBit ) == 0 )
        {
            /* 所需的大小增加了，所以除了请求的字节数之外，它还可以包含一个BlockLink_t结构。 */
            if( xWantedSize > 0 )
            {
                xWantedSize += heapSTRUCT_SIZE;

                /* 确保块始终与所需的字节数对齐。 */
                if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
                {
                    /* Byte alignment required. */
                    xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }

            if( ( xWantedSize > 0 ) && ( xWantedSize <= xFreeBytesRemaining ) )
            {
                /* 从xStart（最低地址）块的列表遍历，直到找到足够的大小。 */
                pxPreviousBlock = &xStart;
                pxBlock = xStart.pxNextFreeBlock;
                while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
                {
                    pxPreviousBlock = pxBlock;
                    pxBlock = pxBlock->pxNextFreeBlock;
                }

                /* 如果达到了结束标记，则没有找到足够大小的块。 */
                if( pxBlock != pxEnd )
                {
                    /* 返回找到的地址空间，跳过一个BlockLink_t结构体大小 */
                    pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + heapSTRUCT_SIZE );

                    /* 这个块被返回使用，所以必须从空闲块链表中取出。 */
                    pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

                    /* 如果该块比需要的大，可以分成两部分。 */
                    if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
                    {
                        /* 这个块被分成两部分。 按照请求的字节数创建一个新的块。 void cast用于防止编译器的字节对齐警告。 */
                        pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );

                        /* 计算从单个块拆分的两个块的大小。*/
                        pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
                        pxBlock->xBlockSize = xWantedSize;

                        /* 将拆分出的新块插入空闲块列表中。 */
                        prvInsertBlockIntoFreeList( ( pxNewBlockLink ) );
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    xFreeBytesRemaining -= pxBlock->xBlockSize;

                    if( xFreeBytesRemaining < xMinimumEverFreeBytesRemaining )
                    {
                        xMinimumEverFreeBytesRemaining = xFreeBytesRemaining;
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    /* 该块被返回 ，标记xBlockSize的最高位，pxNextFreeBlock置NULL */
                    pxBlock->xBlockSize |= xBlockAllocatedBit;
                    pxBlock->pxNextFreeBlock = NULL;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        traceMALLOC( pvReturn, xWantedSize );
    }
    xTaskResumeAll();

    #if( configUSE_MALLOC_FAILED_HOOK == 1 )
    {
        if( pvReturn == NULL )
        {
            extern void vApplicationMallocFailedHook( void );
            vApplicationMallocFailedHook();
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
    #endif

    return pvReturn;
}
/*-----------------------------------------------------------*/

/**
 * 内存释放函数
 *
 * @param pv 待释放内存指针
 */
void ObcMemFree( void *pv )
{
uint8_t *puc = ( uint8_t * ) pv;
BlockLink_t *pxLink;

    if( pv != NULL )
    {
        /* 在释放的内存之前有一个BlockLink_t结构。 */
        puc -= heapSTRUCT_SIZE;

        /* 这种转换是为了防止编译器发出警告。 */
        pxLink = ( void * ) puc;

        /* 检查块是否实际分配。 */
        configASSERT( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 );
        configASSERT( pxLink->pxNextFreeBlock == NULL );

        if( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 )
        {
            if( pxLink->pxNextFreeBlock == NULL )
            {
                /* 该块返回到内存堆，最高位标记该块为空闲 */
                pxLink->xBlockSize &= ~xBlockAllocatedBit;

                vTaskSuspendAll();
                {
                    /* 将此块添加到空闲块的链表中。 */
                    xFreeBytesRemaining += pxLink->xBlockSize;
                    traceFREE( pv, pxLink->xBlockSize );
                    prvInsertBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
                }
                xTaskResumeAll();
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
}
/*-----------------------------------------------------------*/

/**
 * 查询内存堆剩余大小值
 *
 * @return 内存堆剩余大小值
 */
size_t ObcMemGetFreeHeapSize( void )
{
    return xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

/**
 * 查询历史最小空闲内存块大小
 *
 * @return 历史最小空闲内存块大小
 */
size_t ObcMemGetMinimumEverFreeHeapSize( void )
{
    return xMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

void ObcMemInitialiseBlocks( void )
{
    /* This just exists to keep the linker quiet. */
}
/*-----------------------------------------------------------*/

static void prvHeapInit( void )
{
BlockLink_t *pxFirstFreeBlock;
uint8_t *pucHeapEnd, *pucAlignedHeap;

    /* Ensure the heap starts on a correctly aligned boundary. */
    pucAlignedHeap = ( uint8_t * ) ( ( ( portPOINTER_SIZE_TYPE ) &ucHeap[ portBYTE_ALIGNMENT ] ) & ( ( portPOINTER_SIZE_TYPE ) ~portBYTE_ALIGNMENT_MASK ) );

    /* xStart is used to hold a pointer to the first item in the list of free
    blocks.  The void cast is used to prevent compiler warnings. */
    xStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
    xStart.xBlockSize = ( size_t ) 0;

    /* pxEnd is used to mark the end of the list of free blocks and is inserted
    at the end of the heap space. */
    pucHeapEnd = pucAlignedHeap + xTotalHeapSize;
    pucHeapEnd -= heapSTRUCT_SIZE;
    pxEnd = ( void * ) pucHeapEnd;
    configASSERT( ( ( ( uint32_t ) pxEnd ) & ( ( uint32_t ) portBYTE_ALIGNMENT_MASK ) ) == 0UL );
    pxEnd->xBlockSize = 0;
    pxEnd->pxNextFreeBlock = NULL;

    /* To start with there is a single free block that is sized to take up the
    entire heap space, minus the space taken by pxEnd. */
    pxFirstFreeBlock = ( void * ) pucAlignedHeap;
    pxFirstFreeBlock->xBlockSize = xTotalHeapSize - heapSTRUCT_SIZE;
    pxFirstFreeBlock->pxNextFreeBlock = pxEnd;

    /* The heap now contains pxEnd. */
    xFreeBytesRemaining -= heapSTRUCT_SIZE;

    /* Work out the position of the top bit in a size_t variable. */
    xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 );
}
/*-----------------------------------------------------------*/

static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert )
{
BlockLink_t *pxIterator;
uint8_t *puc;

    /* Iterate through the list until a block is found that has a higher address
    than the block being inserted. */
    for( pxIterator = &xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
    {
        /* Nothing to do here, just iterate to the right position. */
    }

    /* Do the block being inserted, and the block it is being inserted after
    make a contiguous block of memory? */
    puc = ( uint8_t * ) pxIterator;
    if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
    {
        pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
        pxBlockToInsert = pxIterator;
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    /* Do the block being inserted, and the block it is being inserted before
    make a contiguous block of memory? */
    puc = ( uint8_t * ) pxBlockToInsert;
    if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
    {
        if( pxIterator->pxNextFreeBlock != pxEnd )
        {
            /* Form one big block from the two blocks. */
            pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
            pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
        }
        else
        {
            pxBlockToInsert->pxNextFreeBlock = pxEnd;
        }
    }
    else
    {
        pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
    }

    /* If the block being inserted plugged a gab, so was merged with the block
    before and the block after, then it's pxNextFreeBlock pointer will have
    already been set, and should not be set here as that would make it point
    to itself. */
    if( pxIterator != pxBlockToInsert )
    {
        pxIterator->pxNextFreeBlock = pxBlockToInsert;
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }
}

