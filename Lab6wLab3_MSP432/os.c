// os.c
// Runs on LM4F120/TM4C123/MSP432
// Lab 3 starter file.
// Daniel Valvano
// March 24, 2016

#include <stdint.h>
#include "os.h"
#include "CortexM.h"
#include "BSP.h"

// function definitions in osasm.s
void StartOS(void);

// function definitions in os.c
void static runperiodicevents(void);
void static runsleep(void);

#define NUMTHREADS  6        // maximum number of threads
#define NUMPERIODIC 2        // maximum number of periodic threads
#define STACKSIZE   100      // number of 32-bit words in stack per thread
struct tcb{
  int32_t *sp;      // pointer to stack (valid for threads not running
  struct tcb *next; // linked-list pointer
	int32_t *blocked;	// pointer to blocked semaphore, nonzero if blocked on this semaphore
	int32_t sleep;		// time to sleep, nonzero if this thread is sleeping
};
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];

struct ptcb{
	void (*task)(void);
	uint32_t period;
	uint32_t counter;
};
typedef struct ptcb ptcbType;
ptcbType PerTask[NUMPERIODIC];

// ******** OS_Init ************
// Initialize operating system, disable interrupts
// Initialize OS controlled I/O: periodic interrupt, bus clock as fast as possible
// Initialize OS global variables
// Inputs:  none
// Outputs: none
void OS_Init(void){
  DisableInterrupts();
  BSP_Clock_InitFastest();// set processor clock to fastest speed
	//OS_AddPeriodicEventThread(&runperiodicevents,1);	//every 1 ms
	BSP_PeriodicTask_Init(runperiodicevents,1000,2);	//Start one HW Timer with periodic interrupt at 1000 Hz
	BSP_PeriodicTask_InitB(runsleep,1000,3);	//Start one HW Timer with periodic interrupt at 1000 Hz
  // perform any initializations needed
}

void SetInitialStack(int i){
	//first set for each stack the stack pointer
	tcbs[i].sp = &Stacks[i][STACKSIZE-16];	//Thread Stack Pointer	R13 = SP
	//fill in bottom positions of the stack with register values, as if thread was already running and interrupted
	Stacks[i][STACKSIZE-1] = 0x01000000; //Thumb bit on last stack element
	//Stacks[i][STACKSIZE-2] = PC; //The Program Counter will be set later with the address of the function it points to, R15 = PC
	Stacks[i][STACKSIZE-3] = 0x14141414; //Initial Link Register dummy value, R14 = LR
	Stacks[i][STACKSIZE-4] = 0x12121212; //R12
	Stacks[i][STACKSIZE-5] = 0x03030303; //R3
	Stacks[i][STACKSIZE-6] = 0x02020202; //R2
	Stacks[i][STACKSIZE-7] = 0x01010101; //R1
	Stacks[i][STACKSIZE-8] = 0x00000000; //R0
	Stacks[i][STACKSIZE-9] = 0x11111111; //R11
	Stacks[i][STACKSIZE-10] = 0x10101010; //R10
	Stacks[i][STACKSIZE-12] = 0x09090909; //R9
	Stacks[i][STACKSIZE-13] = 0x08080808; //R8
  Stacks[i][STACKSIZE-13] = 0x07070707; //R7
  Stacks[i][STACKSIZE-14] = 0x06060606; //R6
  Stacks[i][STACKSIZE-15] = 0x05050505; //R5
  Stacks[i][STACKSIZE-16] = 0x04040404; //R4	
}

//******** OS_AddThreads ***************
// Add six main threads to the scheduler
// Inputs: function pointers to six void/void main threads
// Outputs: 1 if successful, 0 if this thread can not be added
// This function will only be called once, after OS_Init and before OS_Launch
int OS_AddThreads(void(*thread0)(void),
                  void(*thread1)(void),
                  void(*thread2)(void),
                  void(*thread3)(void),
                  void(*thread4)(void),
                  void(*thread5)(void)){
	int32_t sr;	//I bit status
	int32_t i;	//thread index
	sr = StartCritical();	//Disable Interrupts
	
	//initialize TCB circular list
	tcbs[0].next = &tcbs[1];	//main thread 0 points to main thread 1
	tcbs[1].next = &tcbs[2];	//main thread 1 points to main thread 2
	tcbs[2].next = &tcbs[3];	//main thread 2 points to main thread 3	
	tcbs[3].next = &tcbs[4];	//main thread 3 points to main thread 4
	tcbs[4].next = &tcbs[5];	//main thread 4 points to main thread 5
	tcbs[5].next = &tcbs[0];	//main thread 5 points to main thread 0				
	
	//initialize threads as not blocked									
	for(i=0; i< NUMTHREADS; i++){tcbs[i].blocked = 0;}
	
	// initialize RunPt
	RunPt = &tcbs[0];

	// initialize four stacks, including initial PC
	SetInitialStack(0);	//SetInitialStack initial stack of main thread 0
	Stacks[0][STACKSIZE-2] = (int32_t)(thread0);	//Set address of thread0 as PC
	SetInitialStack(1);	//SetInitialStack initial stack of main thread 1
	Stacks[1][STACKSIZE-2] = (int32_t)(thread1);	//Set address of thread1 as PC	
	SetInitialStack(2);	//SetInitialStack initial stack of main thread 2
	Stacks[2][STACKSIZE-2] = (int32_t)(thread2);	//Set address of thread2 as PC
	SetInitialStack(3);	//SetInitialStack initial stack of main thread 3
	Stacks[3][STACKSIZE-2] = (int32_t)(thread3);	//Set address of thread3 as PC
	SetInitialStack(4);	//SetInitialStack initial stack of main thread 4
	Stacks[4][STACKSIZE-2] = (int32_t)(thread4);	//Set address of thread4 as PC
	SetInitialStack(5);	//SetInitialStack initial stack of main thread 5
	Stacks[5][STACKSIZE-2] = (int32_t)(thread5);	//Set address of thread5 as PC	
  

	
	EndCritical(sr);	//Enable Interrupts
	return 1;         // successful
}

//******** OS_AddPeriodicEventThread ***************
// Add one background periodic event thread
// Typically this function receives the highest priority
// Inputs: pointer to a void/void event thread function
//         period given in units of OS_Launch (Lab 3 this will be msec)
// Outputs: 1 if successful, 0 if this thread cannot be added
// It is assumed that the event threads will run to completion and return
// It is assumed the time to run these event threads is short compared to 1 msec
// These threads cannot spin, block, loop, sleep, or kill
// These threads can call OS_Signal
// In Lab 3 this will be called exactly twice
int OS_AddPeriodicEventThread(void(*thread)(void), uint32_t period){
	int32_t static event_number = 0;
	//int32_t frequency;
	//frequency = 1000/period;	//in this case 1000 / 1ms = 1000 Hz
	PerTask[event_number].task = thread;
	PerTask[event_number].period = period;
	PerTask[event_number].counter = 1;
	event_number++;
  return 1;
}

void static runperiodicevents(void){
// **RUN PERIODIC THREADS
	int32_t i;
	for (i=0;i<NUMPERIODIC;i++){	//search for periodic tasks
		PerTask[i].counter--;
		if(PerTask[i].counter == 0) { //it's time to launch a periodic task		
			PerTask[i].task();
			PerTask[i].counter = PerTask[i].period;	//reset
		}
	}
}

void static runsleep(void){
// **DECREMENT SLEEP COUNTERS
	int32_t i;
	for (i=0;i<NUMTHREADS;i++){ if(tcbs[i].sleep != 0) {	//search for sleeping main threads
			tcbs[i].sleep --;	//decrement sleep period by 1ms
		}
	}
}

//******** OS_Launch ***************
// Start the scheduler, enable interrupts
// Inputs: number of clock cycles for each time slice
// Outputs: none (does not return)
// Errors: theTimeSlice must be less than 16,777,216
void OS_Launch(uint32_t theTimeSlice){
  STCTRL = 0;                  // disable SysTick during setup
  STCURRENT = 0;               // any write to current clears it
  SYSPRI3 =(SYSPRI3&0x00FFFFFF)|0xE0000000; // priority 7
  STRELOAD = theTimeSlice - 1; // reload value
  STCTRL = 0x00000007;         // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}
// runs every ms in this project
void Scheduler(void){ // every time slice
	// ROUND ROBIN, skip blocked and sleeping threads
	RunPt = RunPt->next;  // Round Robin point to next thread
	//If next thread is blocked or sleeping, search to the list untill the first unblocked thread
	while((RunPt->blocked != 0)||(RunPt->sleep != 0)){
		RunPt = RunPt->next;
	}
}

//******** OS_Suspend ***************
// Called by main thread to cooperatively suspend operation
// Inputs: none
// Outputs: none
// Will be run again depending on sleep/block status
void OS_Suspend(void){
  STCURRENT = 0;        // any write to current clears it
  INTCTRL = 0x04000000; // trigger SysTick
// next thread gets a full time slice
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
// set sleep parameter in TCB
// suspend, stops running
	RunPt->sleep = sleepTime;
	OS_Suspend();
}

// ******** OS_InitSemaphore ************
// Initialize counting semaphore
// Inputs:  pointer to a semaphore
//          initial value of semaphore
// Outputs: none
void OS_InitSemaphore(int32_t *semaPt, int32_t value){
  //Assign initial value to *semaPt
	*semaPt = value;
}

// ******** OS_Wait ************
// Decrement semaphore and block if less than zero
// Lab2 spinlock (does not suspend while spinning)
// Lab3 block if less than zero
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Wait(int32_t *semaPt){
	DisableInterrupts();
	*semaPt = (*semaPt) - 1;
	if(*semaPt < 0){
		RunPt->blocked = semaPt;	//Point to semaphore which is blocked
		EnableInterrupts();
		OS_Suspend();	//Switch threads by generating a systick interrupt
	}
	EnableInterrupts();
}

// ******** OS_Signal ************
// Increment semaphore
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Signal(int32_t *semaPt){
	tcbType	*threadPt;	//local thread pointer
	DisableInterrupts();
	(*semaPt) = (*semaPt) + 1;
	if(*semaPt <= 0){
		threadPt = RunPt->next;	//point to next thread
		while((threadPt->blocked) != semaPt) {	threadPt = threadPt->next; }//search for a thread that is blocked on this semaphore
		threadPt->blocked = 0;	//unblock 1st blocked thread found
	}
	EnableInterrupts();
}

#define FSIZE 10    // can be any size
uint32_t PutI;      // index of where to put next
uint32_t GetI;      // index of where to get next
uint32_t Fifo[FSIZE];
int32_t CurrentSize;// 0 means FIFO empty, FSIZE means full
uint32_t LostData;  // number of lost pieces of data

// ******** OS_FIFO_Init ************
// Initialize FIFO.  
// One event thread producer, one main thread consumer
// Inputs:  none
// Outputs: none
void OS_FIFO_Init(void){ //Init the FIFO with indexes and CurrentSize and LostData set to 0
	PutI = 0;
	GetI = 0;
	OS_InitSemaphore(&CurrentSize,0);
}

// ******** OS_FIFO_Put ************
// Put an entry in the FIFO.  
// Exactly one event thread puts,
// do not block or spin if full
// Inputs:  data to be stored
// Outputs: 0 if successful, -1 if the FIFO is full
int OS_FIFO_Put(uint32_t data){
	if(CurrentSize == FSIZE) { //FIFO is full
		LostData++;
		return -1; //Error
	}
	else {
		Fifo[PutI] = data;	//store data in FIFO at put index
		PutI = (PutI + 1)%FSIZE; //Increment Put index and wrap around if necessary
		OS_Signal(&CurrentSize);
		return 0;	//Success
	}
}

// ******** OS_FIFO_Get ************
// Get an entry from the FIFO.   
// Exactly one main thread get,
// do block if empty
// Inputs:  none
// Outputs: data retrieved
uint32_t OS_FIFO_Get(void){uint32_t data;
	OS_Wait(&CurrentSize);	//Wait till there is data in FIFO, block if empty
	data = Fifo[GetI];	//Get stored data from Fifo
	GetI = (GetI + 1) % FSIZE;	//Incremet Get index and wrap around
  return data;
}
