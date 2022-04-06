#include "mem.h"
#include "stdlib.h"
#include "string.h"
#include <pthread.h>
#include <stdio.h>

static BYTE _ram[RAM_SIZE];

static struct {
	uint32_t proc;	// ID of process currently uses this page
	int index;	// Index of the page in the list of pages allocated
			// to the process.
	int next;	// The next page in the list. -1 if it is the last
			// page.
} _mem_stat [NUM_PAGES];

static pthread_mutex_t mem_lock;

void init_mem(void) {
	memset(_mem_stat, 0, sizeof(*_mem_stat) * NUM_PAGES);
	memset(_ram, 0, sizeof(BYTE) * RAM_SIZE);
	pthread_mutex_init(&mem_lock, NULL);
}

/* get offset of the virtual address */
static addr_t get_offset(addr_t addr) {
	return addr & ~((~0U) << OFFSET_LEN);
}

/* get the first layer index */
static addr_t get_first_lv(addr_t addr) {
	return addr >> (OFFSET_LEN + PAGE_LEN);
}

/* get the second layer index */
static addr_t get_second_lv(addr_t addr) {
	return (addr >> OFFSET_LEN) - (get_first_lv(addr) << PAGE_LEN);
}

/* Search for page table table from the a segment table */
static struct page_table_t * get_page_table(
		addr_t index, 	// Segment level index
		struct seg_table_t * seg_table) { // first level table
	int i;
	for (i = 0; i < seg_table->size; i++) {
		if (index==seg_table->table[i].v_index) return seg_table->table[i].pages;
	}
	return NULL;

}

/* Translate virtual address to physical address. If [virtual_addr] is valid,
 * return 1 and write its physical counterpart to [physical_addr].
 * Otherwise, return 0 */
static int translate(
		addr_t virtual_addr, 	// Given virtual address
		addr_t * physical_addr, // Physical address to be returned
		struct pcb_t * proc) {  // Process uses given virtual address

	/* Offset of the virtual address */
	addr_t offset = get_offset(virtual_addr);
	/* The first layer index */
	addr_t first_lv = get_first_lv(virtual_addr);
	/* The second layer index */
	addr_t second_lv = get_second_lv(virtual_addr);
	
	/* Search in the first level */
	struct page_table_t * page_table = NULL;
	page_table = get_page_table(first_lv, proc->seg_table);
	if (page_table == NULL) {
		return 0;
	}

	int i;
	for (i = 0; i < page_table->size; i++) {
		if (page_table->table[i].v_index == second_lv) {
			*physical_addr=((page_table->table[i].p_index)<<OFFSET_LEN)+offset;
			return 1;
		}
	}
	return 0;	
}

addr_t alloc_mem(uint32_t size, struct pcb_t * proc) {
	pthread_mutex_lock(&mem_lock);
	addr_t ret_mem = 0;
	uint32_t num_pages = (size % PAGE_SIZE) ? size / PAGE_SIZE :
		size / PAGE_SIZE + 1; // Number of pages we will use
	int mem_avail = 0; // We could allocate new memory region or not?

	//--------------- KIEM TRA PHYSICAL MEMORY----------------
	int k, spaces=0;   // If proc = 0, the page is free and the OS could allocated it to any process
	for (int k=0;k<NUM_PAGES;k++){
		if (spaces==num_pages){
			mem_avail=1;
			break;
		}
		else {
			if(_mem_state[k].proc==0) spaces++;
		}
	}
	//-------------------------------------------------------

	//--------------- KIEM TRA VIRTUAL MEMORY----------------
	if (pro->bp+numpages*PAGE_SIZE>(1<<ADDRESS_SIZE)) mem_avail=0;
	//-------------------------------------------------------

	if (mem_avail) {
		ret_mem = proc->bp;
		proc->bp += num_pages*PAGE_SIZE;
		addr_t addr_vir=ret_mem;
		int seg_index=-2,page_size=-2,pre_frame=-2;
		spaces=0;
		for (k=0;k<NUM_PAGES;k++){
			if (_mem_stat[k].proc==0){
				_mem_stat[k].proc=proc->pid;
				_mem_stat[k].index=spaces;
				if (pre_frame!=-2) _mem_stat[pre_frame].next=k;
				pre_frame=k;
				seg_index=get_first_lv(addr_vir);

				if (proc->seg_table->table[seg_index].pages==NULL){
					proc->seg_table->table[seg_index].pages=malloc(sizeof(struct page_table_t));
					proc->seg_table->table[seg_index].pages->size=0;
				}	
				
				proc->seg_table->table[seg_index].pages->size+=1;
				page_size=proc->seg_table->table[seg_index].pages->size-1;
				proc->seg_table->table[seg_index].v_index=seg_index;
				proc->seg_table->table[seg_index].pages->table[page_size].v_index=get_second_lv(addr_vir);
				proc->seg_table->table[seg_index].pages->table[page_size].p_index=k;
				
				addr_vir+=PAGE_SIZE;
				spaces+=1;
				proc->seg_table->size+=1;
				if (spaces==num_pages){
					_mem_stat[k].next=-1;
					break;
				}
			}
		}
	}
	pthread_mutex_unlock(&mem_lock);
	return ret_mem;
}

int free_mem(addr_t address, struct pcb_t * proc) {
	pthread_mutex_lock(&mem_lock);
	int numpages=0,i;
	addr_t addr_phy,addr_vir=address;
	if (translate(address,&addr_phy,proc)){
		addr_t page_phy=addr_phy>>OFFSET_LEN;
		while (page_phy!=-1){
			_mem_stat[page_phy].proc=0;
			addr_t seg_index=get_first_lv(addr_vir);
			for (i=0;i<proc->seg_table->table[seg_index].pages->size;i++){
				if (proc->seg_table->table[seg_index].pages->table[i].p_index==page_phy){
					proc->seg_table->table[segIndex].pages->table[i].v_index=0;
					proc->seg_table->table[segIndex].pages->table[i].p_index=0;
				}
			}
			page_phy=_mem_stat[page_phy].next;
			addr_vir+=PAGE_SIZE;
		}
	}
	pthread_mutex_unlock(&mem_lock);
	return 0;
}

int read_mem(addr_t address, struct pcb_t * proc, BYTE * data) {
	addr_t physical_addr;
	if (translate(address, &physical_addr, proc)) {
		*data = _ram[physical_addr];
		return 0;
	}else{
		return 1;
	}
}

int write_mem(addr_t address, struct pcb_t * proc, BYTE data) {
	addr_t physical_addr;
	if (translate(address, &physical_addr, proc)) {
		_ram[physical_addr] = data;
		return 0;
	}else{
		return 1;
	}
}

void dump(void) {
	int i;
	for (i = 0; i < NUM_PAGES; i++) {
		if (_mem_stat[i].proc != 0) {
			printf("%03d: ", i);
			printf("%05x-%05x - PID: %02d (idx %03d, nxt: %03d)\n",
				i << OFFSET_LEN,
				((i + 1) << OFFSET_LEN) - 1,
				_mem_stat[i].proc,
				_mem_stat[i].index,
				_mem_stat[i].next
			);
			int j;
			for (	j = i << OFFSET_LEN;
				j < ((i+1) << OFFSET_LEN) - 1;
				j++) {
				
				if (_ram[j] != 0) {
					printf("\t%05x: %02x\n", j, _ram[j]);
				}
					
			}
		}
	}
}
