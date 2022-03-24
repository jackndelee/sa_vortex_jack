#include <stdint.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <unistd.h>
#include <assert.h> //assert 매크로, 정해진 조건이 맞지않을때 프로그램 중단. false ->stop, ture -> keep
#include <cmath>
#include <sstream>
#include <unordered_map>
#include <list> //c++

#if defined(USE_FPGA) || defined(USE_ASE) 
#include <opae/fpga.h>
#include <uuid/uuid.h>


#elif defined(USE_VLSIM)
#include <fpga.h>
#endif

#include <vortex.h>
#include <VX_config.h>
#include "vortex_afu.h"

#ifdef SCOPE
#include "vx_scope.h"
#endif

#define CHECK_RES(_expr)                                \
   do {                                                 \
     fpga_result res = _expr;                           \
     if (res == FPGA_OK)                                \
       break;                                           \
     printf("[VXDRV] Error: '%s' returned %d, %s!\n",   \
            #_expr, (int)res, fpgaErrStr(res));         \
     return -1;                                         \
   } while (false)



///////////////////////////////////////////////////////////////////////////////

#define CMD_MEM_READ        AFU_IMAGE_CMD_MEM_READ
#define CMD_MEM_WRITE       AFU_IMAGE_CMD_MEM_WRITE
#define CMD_RUN             AFU_IMAGE_CMD_RUN

#define MMIO_CMD_TYPE       (AFU_IMAGE_MMIO_CMD_TYPE * 4)
#define MMIO_IO_ADDR        (AFU_IMAGE_MMIO_IO_ADDR * 4)
#define MMIO_MEM_ADDR       (AFU_IMAGE_MMIO_MEM_ADDR * 4)
#define MMIO_DATA_SIZE      (AFU_IMAGE_MMIO_DATA_SIZE * 4)
#define MMIO_DEV_CAPS       (AFU_IMAGE_MMIO_DEV_CAPS * 4)
#define MMIO_STATUS         (AFU_IMAGE_MMIO_STATUS * 4)

///////////////////////////////////////////////////////////////////////////////




typedef struct vx_device_ {
    fpga_handle fpga;
    size_t mem_allocation;
    unsigned version;
    unsigned num_cores;
    unsigned num_warps;
    unsigned num_threads;
} vx_device_t;



typedef struct vx_buffer_ {
    uint64_t wsid;
    void* host_ptr;
    uint64_t io_addr;
    vx_device_h hdevice;
    size_t size;
} vx_buffer_t;


inline size_t align_size(size_t size, size_t alignment) {        //말 그대로 size
    assert(0 == (alignment & (alignment - 1)));
    return (size + alignment - 1) & ~(alignment - 1);
}

inline bool is_aligned(size_t addr, size_t alignment) {    // bool  // bit value // addr 값과
    assert(0 == (alignment & (alignment - 1)));
    return 0 == (addr & (alignment - 1));
}

///////////////////////////////////////////////////////////////////////////////

#ifdef DUMP_PERF_STATS
class AutoPerfDump {
private:
    std::list<vx_device_h> devices_; // vx_device_h 리스트

public:
    AutoPerfDump() {} 

    ~AutoPerfDump() {
        for (auto device : devices_) {
            vx_dump_perf(device, stdout);
        }
    }

    void add_device(vx_device_h device) {
        devices_.push_back(device);
    }

    void remove_device(vx_device_h device) {
        devices_.remove(device);
    }    
};

AutoPerfDump gAutoPerfDump;
#endif

///////////////////////////////////////////////////////////////////////////////



extern int vx_dev_caps(vx_device_h hdevice, unsigned caps_id, unsigned *value) {
    // device configurations

    printf("vx_dev_caps start\n");
    if (nullptr == hdevice)
        return -1;

    vx_device_t *device = ((vx_device_t*)hdevice);

    printf("caps_id : %d\n", caps_id);
    switch (caps_id) {
    case VX_CAPS_VERSION:
        *value = device->version; //0
        break;
    case VX_CAPS_MAX_CORES:
        *value = device->num_cores; //1
        break;
    case VX_CAPS_MAX_WARPS:
        *value = device->num_warps; //2
        break;
    case VX_CAPS_MAX_THREADS:
        *value = device->num_threads; //3
        break;
    case VX_CAPS_CACHE_LINE_SIZE:
        *value = CACHE_BLOCK_SIZE; //4
        break;
    case VX_CAPS_LOCAL_MEM_SIZE:
        *value = LOCAL_MEM_SIZE; //5
        break;
    case VX_CAPS_ALLOC_BASE_ADDR:
        *value = ALLOC_BASE_ADDR; //6
        break;
    case VX_CAPS_KERNEL_BASE_ADDR: //7
          *value = STARTUP_ADDR; 
        break;

    default:
        fprintf(stderr, "[VXDRV] Error: invalid caps id: %d\n", caps_id);
        std::abort();
        return -1;
    }
    
    printf("vx_dev_caps end\n");
    return 0;
   
}




extern int vx_dev_open(vx_device_h* hdevice) { //pocl에서 확인하니 빈 구조체,주소가 넘어옴 
                                               
    printf("vx_dev_open start, device adress X, struct address : %p\n", hdevice);
    
    if (nullptr == hdevice)
        return  -1;

    fpga_handle accel_handle;    
    vx_device_t* device;   


#ifndef USE_VLSIM
   fpga_result res;    
    fpga_token accel_token;
    fpga_properties filter = nullptr;    
    fpga_guid guid; 
    uint32_t num_matches;



    // Set up a filter that will search for an accelerator
    CHECK_RES(fpgaGetProperties(nullptr, &filter));
     printf("vx_dev_open ======= fpgaGetProperties\n"); 
    res = fpgaPropertiesSetObjectType(filter, FPGA_ACCELERATOR);
     printf("vx_dev_open ======= fpgaPropertiesSetObjectType\n");  

    if (res != FPGA_OK) {
        fprintf(stderr, "[VXDRV] Error: fpgaGetProperties() returned %d, %s!\n", (int)res, fpgaErrStr(res));
        fpgaDestroyProperties(&filter);
       printf("vx_dev_open ======= fpgaPropertiesSetObjectType\n");  
        return -1;
    }

    // Add the desired UUID to the filter
    uuid_parse(AFU_ACCEL_UUID, guid);
    res = fpgaPropertiesSetGUID(filter, guid);
    printf("vx_dev_open ======= fpgaPropertiesSetGUID\n"); 

    if (res != FPGA_OK) {
        fprintf(stderr, "[VXDRV] Error: fpgaPropertiesSetGUID() returned %d, %s!\n", (int)res, fpgaErrStr(res));
        fpgaDestroyProperties(&filter);
         printf("vx_dev_open ======= fpgaDestroyProperties\n"); 

        return -1;
    }

    // Do the search across the available FPGA contexts
    num_matches = 1;
    res = fpgaEnumerate(&filter, 1, &accel_token, 1, &num_matches);
    printf("vx_dev_open ======= fpgaEnumerate\n"); 

    if (res != FPGA_OK) {
        fprintf(stderr, "[VXDRV] Error: fpgaEnumerate() returned %d, %s!\n", (int)res, fpgaErrStr(res));
        fpgaDestroyProperties(&filter);
        printf("vx_dev_open ======= fpgaDestroyProperties\n");
        return -1;
    }

    // Not needed anymore
    fpgaDestroyProperties(&filter);
    printf("vx_dev_open ======= fpgaDestroyProperties\n");

    if (num_matches < 1) {
        fprintf(stderr, "[VXDRV] Error: accelerator %s not found!\n", AFU_ACCEL_UUID);
        fpgaDestroyToken(&accel_token);
        printf("vx_dev_open ======= fpgaDestroyToken\n");
        
        return -1;
    }

    // Open accelerator
    res = fpgaOpen(accel_token, &accel_handle, 0);
     printf("vx_dev_open ======= fpgaOpen\n");
     
    if (res != FPGA_OK) {
        fprintf(stderr, "[VXDRV] Error: fpgaOpen() returned %d, %s!\n", (int)res, fpgaErrStr(res));
        fpgaDestroyToken(&accel_token);
        printf("vx_dev_open ======= fpgaDestroyToken\n");
        return -1;
    }

    // Done with token
    fpgaDestroyToken(&accel_token);
    printf("vx_dev_open ======= fpgaDestroyToken\n");
#else
    // Open accelerator
    CHECK_RES(fpgaOpen(NULL, &accel_handle, 0));
     printf("vx_dev_open ======= fpgaOpen\n");
#endif

    // allocate device object
    device = (vx_device_t*)malloc(sizeof(vx_device_t));
    printf("vx_dev_open ======= allocate device object\n");

    if (nullptr == device) {
        fpgaClose(accel_handle);
        return -1;
    }
   
    //포인터로 구조체/공용체 멤버 접근
    device->fpga = accel_handle;
    device->mem_allocation = ALLOC_BASE_ADDR;
    
    {   
        // Load device CAPS
        uint64_t dev_caps; //64비트 어떤값이 저장
        int ret = fpgaReadMMIO64(device->fpga, 0, MMIO_DEV_CAPS, &dev_caps); 
        printf("vx_dev_open ====before === fpgaReadMMIO64 === version=%d, num_cores=%d, num_warps=%d, num_threads=%d\n", 
                device->version, device->num_cores, device->num_warps, device->num_threads);
     
        if (ret != FPGA_OK) {
            fpgaClose(accel_handle);
            return ret;
        }
        device->version     = (dev_caps >> 0)  & 0xffff; //bit mask
        device->num_cores   = (dev_caps >> 16) & 0xffff;
        device->num_warps   = (dev_caps >> 32) & 0xffff;
        device->num_threads = (dev_caps >> 48) & 0xffff;
      
       printf("vx_dev_open ====after ==== fpgaReadMMIO64 === version=%d, num_cores=%d, num_warps=%d, num_threads=%d\n", 
                device->version, device->num_cores, device->num_warps, device->num_threads);
     



    #ifndef NDEBUG    
        fprintf(stdout, "[VXDRV] DEVCAPS: version=%d, num_cores=%d, num_warps=%d, num_threads=%d\n", 
                device->version, device->num_cores, device->num_warps, device->num_threads);
    #endif
    }
    
#ifdef SCOPE
    {
        int ret = vx_scope_start(accel_handle, 0, -1);
        {printf("vx_scope_start\n");}
        if (ret != 0) {
            fpgaClose(accel_handle);
            return ret;
        }
    }
#endif    
   //포인터 -> 구조체 덮어씌움
    *hdevice = device;
    // device->version, device->num_cores, device->num_warps, device->num_threads)




#ifdef DUMP_PERF_STATS
    gAutoPerfDump.add_device(*hdevice);
#endif

    printf("vx_dev_open end\n");
    return 0;
      
}








extern int vx_dev_close(vx_device_h hdevice) {
    printf("vx_dev_close_start\n");

    if (nullptr == hdevice)
        return -1;

    vx_device_t *device = ((vx_device_t*)hdevice);

#ifdef SCOPE
    vx_scope_stop(device->fpga);
    {
    printf("vx_scope_stop\n"); }

#endif


#ifdef DUMP_PERF_STATS
    gAutoPerfDump.remove_device(hdevice);
    vx_dump_perf(hdevice, stdout);{
        printf("vx_dump_perf\n");    }
    
#endif

    fpgaClose(device->fpga);

    printf("vx_dev_close end\n");
    return 0;

    
}








extern int vx_alloc_dev_mem(vx_device_h hdevice, size_t size, size_t* dev_maddr) {
    printf("vx_alloc_dev_mem  start, actual device struct address : %p \n", hdevice);

    if (nullptr == hdevice 
     || nullptr == dev_maddr
     || 0 >= size)
        return -1;

    vx_device_t *device = ((vx_device_t*)hdevice);
    
    size_t dev_mem_size = LOCAL_MEM_SIZE; // vortex.h : #define LOCAL_MEM_SIZE   0xffffffff // 32 bit
    size_t asize = align_size(size, CACHE_BLOCK_SIZE); // vortex.h : #define CACHE_BLOCK_SIZE 64
    
    if (device->mem_allocation + asize > dev_mem_size) 
            return -1;   
            
    // 실제 device mem 할당 
    *dev_maddr = device->mem_allocation;
       printf("vx_alloc_dev_mem , actual device adress : %p,  dev_maddr: %x \n", hdevice, *dev_maddr);
    device->mem_allocation += asize;

   printf("vx_alloc_dev_mem end\n");

    return 0;

  


}





extern int vx_alloc_shared_mem(vx_device_h hdevice, size_t size, vx_buffer_h* hbuffer) {
    printf("vx_alloc_shared_mem start, device adress : %p\n", hdevice);

    fpga_result res;
    void* host_ptr; // Virtual Addr of pointer
    uint64_t wsid; // buffer workspace ID
    uint64_t io_addr; // Pointer to mem where IO addr
    vx_buffer_t* buffer;

    if (nullptr == hdevice
     || 0 >= size
     || nullptr == hbuffer)
        return -1;

    vx_device_t *device = ((vx_device_t*)hdevice);

    size_t asize = align_size(size, CACHE_BLOCK_SIZE);

    res = fpgaPrepareBuffer(device->fpga, asize, &host_ptr, &wsid, 0);
    //    fpgaPrepareBuffer(fpga_handle handle, uint64_t len, void **buf_addr, uint64_t *wsid, int flags)

    uint64_t* temp = (uint64_t*)host_ptr;
     printf("vx_alloc_shared_mem ======= fpgaPrepareBuffer wsid: %d, host_ptr addr: %p, host_ptr value: %x\n", wsid, host_ptr, *temp); 


    if (FPGA_OK != res) {
        return -1;
    }

    // Get the physical address of the buffer in the accelerator
    res = fpgaGetIOAddress(device->fpga, wsid, &io_addr);
        //fpgaGetIOAddress(fpga_handle handle, uint64_t wsid, uint64_t *iova)
         printf("vx_alloc_shared_mem ======= fpgaGetIOAddress wsid: %d, io_addr: %x\n", wsid, io_addr);

    if (FPGA_OK != res) {
        fpgaReleaseBuffer(device->fpga, wsid);
         printf("vx_alloc_shared_mem ======= fpgaReleaseBuffer wsid: %d, io_addr: %x\n", wsid, io_addr);
        return -1;
    }

    // allocate buffer object
    buffer = (vx_buffer_t*)malloc(sizeof(vx_buffer_t));

    if (nullptr == buffer) {
        fpgaReleaseBuffer(device->fpga, wsid);
          printf("vx_alloc_shared_mem ======= fpgaReleaseBuffer wsid: %d\n", wsid);
        return -1;
    }



    // buffer 
    buffer->wsid     = wsid;
    buffer->host_ptr = host_ptr;
    buffer->io_addr  = io_addr;
    buffer->hdevice  = hdevice;
    buffer->size     = asize;

    *hbuffer = buffer;
     


    printf("vx_alloc_shared_mem end\n");      
    return 0;

   
}






extern void* vx_host_ptr(vx_buffer_h hbuffer) {

  

    if (nullptr == hbuffer)
        return nullptr;

    vx_buffer_t* buffer = ((vx_buffer_t*)hbuffer);
   
   printf("vx_host_ptr \n");
   
    return buffer->host_ptr;
   
// 함수의 의미 파악 /.
// vx_buffer_h -> 왜 굳이 만들었나
// vx_buffer_t 를 pocl에서 찾아보면 없음 -> vx_buffer_h -> 접근을 위한 전용 함수
}





extern int vx_buf_release(vx_buffer_h hbuffer) {

    printf("vx_buf_release start\n");
    if (nullptr == hbuffer)
        return -1;

    vx_buffer_t* buffer = ((vx_buffer_t*)hbuffer);
    vx_device_t *device = ((vx_device_t*)buffer->hdevice);

    // printf("vx_buf_release : Before vx_buffer == wsid: %p, io_addr: %p\n",wsid, io_addr);
    fpgaReleaseBuffer(device->fpga, buffer->wsid);

    free(buffer);
     
   // printf("vx_buf_release : After vx_buffer == wsid: %p, io_addr: %p\n",wsid, io_addr);
         
    printf("vx_buf_release end\n"); 
    return 0;
}






extern int vx_ready_wait(vx_device_h hdevice, long long timeout) { //buffer 의 sleep, wake 기능

    printf("vx_ready_wait start======== device address : %x, timeout : %ld\n", hdevice, timeout);

    if (nullptr == hdevice)
        return -1;

    std::unordered_map<int, std::stringstream> print_bufs; // c++ STL 중의 하나인 unorderd_map 사용
    
    vx_device_t *device = ((vx_device_t*)hdevice);

    struct timespec sleep_time;  /// 보통 sleep time 은 buffer가 비어있는 시간을 보기위해 

#if defined(USE_ASE)
    sleep_time.tv_sec = 1;
    sleep_time.tv_nsec = 0;
#else 
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 1000000;
#endif

    // to milliseconds
    long long sleep_time_ms = (sleep_time.tv_sec * 1000) + (sleep_time.tv_nsec / 1000000); // (0 * 1000) + (1000000/1000000) = 1
    printf("vx_ready_wait start======== timeout : %ld\n", timeout);

    for (;;) {
        uint64_t status;
        CHECK_RES(fpgaReadMMIO64(device->fpga, 0, MMIO_STATUS, &status)); //MMIO_STATUS =  uint64_t offset //register offset
        printf("vx_ready_wait ======= fpgaReadMMIO64--  MMIO_STATUS: %x, status: %d\n",MMIO_STATUS, status); //status : result

        uint16_t cout_data = (status >> 8) & 0xffff;
        if (cout_data & 0x0001) {  //cout data: bit value
            do {
                char cout_char = (cout_data >> 1) & 0xff;
                int cout_tid = (cout_data >> 9) & 0xff;
                auto& ss_buf = print_bufs[cout_tid];
                ss_buf << cout_char;
                if (cout_char == '\n') {
                    std::cout << std::dec << "#" << cout_tid << ": " << ss_buf.str() << std::flush;
                    ss_buf.str("");
                }
                CHECK_RES(fpgaReadMMIO64(device->fpga, 0, MMIO_STATUS, &status));
                printf("vx_ready_wait ======= fpgaReadMMIO64--in do_while--  MMIO_STATUS: %x, status: %d\n",MMIO_STATUS, &status);
                cout_data = (status >> 8) & 0xffff;
            } while (cout_data & 0x0001);
        }

        uint8_t state = status & 0xff; //
       printf("vx_ready_wait start======== state : %x\n", state);


        if (0 == state || 0 == timeout) {
            for (auto& buf : print_bufs) {
                auto str = buf.second.str(); ////초기화 한 값에 따라서 알아서 자료형 결정
                if (!str.empty()) {
                std::cout << "#" << buf.first << ": " << str << std::endl;
                }
            }
            if (state != 0) {
                fprintf(stdout, "[VXDRV] ready-wait timed out: state=%d\n", state);
            }
            break;
        }

        nanosleep(&sleep_time, nullptr);
        timeout -= sleep_time_ms; // -1 // 다시 올라가서 
    };
    
    printf("vx_ready_wait end\n");
    return 0;

    
}








extern int vx_copy_to_dev(vx_buffer_h hbuffer, size_t dev_maddr, size_t size, size_t src_offset) {

    printf("vx_copy_to_dev start\n");

    if (nullptr == hbuffer 
     || 0 >= size)
        return -1;

    vx_buffer_t *buffer = ((vx_buffer_t*)hbuffer);
    vx_device_t *device = ((vx_device_t*)buffer->hdevice);
  
      printf("vx_copy_to_dev start======= dev_maddr start: %x\n",dev_maddr);


    size_t dev_mem_size = LOCAL_MEM_SIZE; 
    size_t asize = align_size(size, CACHE_BLOCK_SIZE);



    // check alignment memory
    if (!is_aligned(dev_maddr, CACHE_BLOCK_SIZE))
        return -1;
    if (!is_aligned(buffer->io_addr + src_offset, CACHE_BLOCK_SIZE))
        return -1;

 

    // bound checking
    if (src_offset + asize > buffer->size)
        return -1;
    if (dev_maddr + asize > dev_mem_size)
        return -1;

    // Ensure ready for new command
    if (vx_ready_wait(buffer->hdevice, -1) != 0)
        return -1;

        
    auto ls_shift = (int)std::log2(CACHE_BLOCK_SIZE); //log2 사용,  auto 사용 // =6   //  bus, data size 맞추는 느낌 
    
    
    printf("vx_copy_to_dev start======= fpgaWriteMMIO64 before\n");
    
    /*
    printf("vx_copy_to_dev====== fpgaWriteMMIO64===before==== dev_maddr + MMIO_IO_ADDR: %x\n", dev_maddr + MMIO_IO_ADDR);
    printf("vx_copy_to_dev====== fpgaWriteMMIO64===before==== dev_maddr + MMIO_MEM_ADDR: %x\n", dev_maddr + MMIO_MEM_ADDR);
    printf("vx_copy_to_dev====== fpgaWriteMMIO64===before==== dev_maddr + MMIO_DATA_SIZE: %x\n", dev_maddr + MMIO_DATA_SIZE);
    printf("vx_copy_to_dev====== fpgaWriteMMIO64===before==== dev_maddr + MMIO_CMD_TYPE: %x\n", dev_maddr + MMIO_CMD_TYPE);
    */
    

    CHECK_RES(fpgaWriteMMIO64(device->fpga, 0, MMIO_IO_ADDR, (buffer->io_addr + src_offset) >> ls_shift)); //cmd 
    CHECK_RES(fpgaWriteMMIO64(device->fpga, 0, MMIO_MEM_ADDR, dev_maddr >> ls_shift));
    CHECK_RES(fpgaWriteMMIO64(device->fpga, 0, MMIO_DATA_SIZE, asize >> ls_shift));   
    CHECK_RES(fpgaWriteMMIO64(device->fpga, 0, MMIO_CMD_TYPE, CMD_MEM_WRITE));
    
    /*
    printf("vx_copy_to_dev====== fpgaWriteMMIO64===after==== dev_maddr + MMIO_IO_ADDR: %x\n", dev_maddr + MMIO_IO_ADDR);
    printf("vx_copy_to_dev====== fpgaWriteMMIO64===after==== dev_maddr + MMIO_MEM_ADDR: %x\n", dev_maddr + MMIO_MEM_ADDR);
    printf("vx_copy_to_dev====== fpgaWriteMMIO64===after==== dev_maddr + MMIO_DATA_SIZE: %x\n", dev_maddr + MMIO_DATA_SIZE);
    printf("vx_copy_to_dev====== fpgaWriteMMIO64===after==== dev_maddr + MMIO_CMD_TYPE: %x\n", dev_maddr + MMIO_CMD_TYPE);
    */
    printf("vx_copy_to_dev end======= fpgaWriteMMIO64 after\n");


    // Wait for the write operation to finish
    if (vx_ready_wait(buffer->hdevice, -1) != 0)
        return -1;


    printf("vx_copy_to_dev end======= dev_maddr end: %x\n",dev_maddr);
     


    printf("vx_copy_to_dev end\n");
    return 0;
    
}






extern int vx_copy_from_dev(vx_buffer_h hbuffer, size_t dev_maddr, size_t size, size_t dest_offset) {

    printf("vx_copy_from_dev start\n");
    if (nullptr == hbuffer 
     || 0 >= size)
        return -1;

    vx_buffer_t *buffer = ((vx_buffer_t*)hbuffer);
    vx_device_t *device = ((vx_device_t*)buffer->hdevice);

     printf("vx_copy_from_dev start======= dev_maddr start: %x\n",dev_maddr);

    size_t dev_mem_size = LOCAL_MEM_SIZE;  
    size_t asize = align_size(size, CACHE_BLOCK_SIZE);

    // check alignment
    if (!is_aligned(dev_maddr, CACHE_BLOCK_SIZE))
        return -1;
    if (!is_aligned(buffer->io_addr + dest_offset, CACHE_BLOCK_SIZE))
        return -1; 

    // bound checking
    if (dest_offset + asize > buffer->size)
        return -1;
    if (dev_maddr + asize > dev_mem_size)
        return -1;

    // Ensure ready for new command
    if (vx_ready_wait(buffer->hdevice, -1) != 0)
     
        return -1;
          printf("vx_ready_wait 1st call\n");

    auto ls_shift = (int)std::log2(CACHE_BLOCK_SIZE);


    printf("vx_copy_from_dev====== fpgaWriteMMIO64===before==== dev_maddr: %x\n", dev_maddr);

   //fpgaWriteMMIO64 (fpga_handle handle , uint32_t mmio_num , uint64_t offset , uint64_t value )
    CHECK_RES(fpgaWriteMMIO64(device->fpga, 0, MMIO_IO_ADDR, (buffer->io_addr + dest_offset) >> ls_shift));
    CHECK_RES(fpgaWriteMMIO64(device->fpga, 0, MMIO_MEM_ADDR, dev_maddr >> ls_shift));    
    CHECK_RES(fpgaWriteMMIO64(device->fpga, 0, MMIO_DATA_SIZE, asize >> ls_shift));   
    CHECK_RES(fpgaWriteMMIO64(device->fpga, 0, MMIO_CMD_TYPE, CMD_MEM_READ));
    /// COMMAND 만드는 역할



    printf("vx_copy_from_dev====== fpgaWriteMMIO64===before====dev_maddr: %x\n", dev_maddr);

    // Wait for the write operation to finish
    if (vx_ready_wait(buffer->hdevice, -1) != 0)
            return -1;
     printf("vx_ready_wait 2nd call\n");
     
    printf("vx_copy_from_dev start======= dev_maddr end: %x\n",dev_maddr);



     printf("vx_copy_from_dev end\n");
    return 0;
    
}






extern int vx_start(vx_device_h hdevice) {
    printf("vx_start start\n");
    
    if (nullptr == hdevice)
        return -1;   

    vx_device_t *device = ((vx_device_t*)hdevice);

    // Ensure ready for new command
    if (vx_ready_wait(hdevice, -1) != 0)
        return -1;    
  

   // printf("fpgaWriteMMIO64===before==== mem_allocation: %p\n", mem_allocation);

    // start execution    
    CHECK_RES(fpgaWriteMMIO64(device->fpga, 0, MMIO_CMD_TYPE, CMD_RUN));

    // printf("fpgaWriteMMIO64===after==== mem_allocation: %p\n",  mem_allocation);


     printf("vx_start end\n");
    return 0;
    
}