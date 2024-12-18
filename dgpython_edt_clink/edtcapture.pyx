import os
import sys
import threading
import atexit 

from dataguzzler_python import dgpy
from dataguzzler_python.dynamic_metadata import DynamicMetadata
from dataguzzler_python.dgpy import InitCompatibleThread

import spatialnde2 as snde 

#from libcpp cimport bool as bool_t
#from libcpp.string cimport string
from cython.operator cimport dereference as deref
from cpython.buffer cimport PyObject_GetBuffer,PyBuffer_Release,PyBUF_ANY_CONTIGUOUS,PyBUF_SIMPLE
from cpython cimport Py_DECREF,PyObject,PyBytes_FromStringAndSize

from libc.stdio cimport fprintf,stderr

from libc.stdint cimport uint64_t
from libc.stdint cimport int64_t
from libc.stdint cimport int32_t
from libc.stdint cimport uint32_t
from libc.stdint cimport int16_t
from libc.stdint cimport uint16_t
from libc.stdint cimport int8_t
from libc.stdint cimport uint8_t
from libc.stdint cimport uintptr_t
from libc.errno cimport errno,EAGAIN,EINTR
from libc.stdlib cimport malloc,calloc,free
from libc.string cimport memcpy

import numpy as np
cimport numpy as np
from numpy cimport NPY_SHORT,PyArray_New,import_array,npy_intp

import_array()

EDT_ENABLE_FAST_REARM=True # basically a compile time variable

cdef extern from "edtcapture_c.h" nogil:
    void convert_edt_image(unsigned char* gotbuf,unsigned char* recdata, unsigned bytes_per_pixel, unsigned ImgWidth, unsigned ImgHeight, int discardtopline,int FlipLR, int FlipUD)

cdef extern from "edtinc.h" nogil:

    ctypedef void *EdtDev
    ctypedef EdtDev PdvDev
    ctypedef unsigned uint_t
    ctypedef enum EdtDmaWaitStatus: EDT_WAIT_OK, EDT_WAIT_TIMEOUT

    extern unsigned PDV_CL_CFG2
    extern unsigned PDV_CL_CFG2_FAST_REARM
    extern uint_t PDV_STAT
    extern uint_t PDV_OVERRUN
    #extern int EDT_WAIT_OK
    #extern int EDT_WAIT_USER_WAKEUP
    #extern int EDT_WAIT_TIMEOUT
    
    PdvDev pdv_open_device(const char *edt_devname, int unit, int channel, int verbose)
    PdvDev pdv_open_channel(const char *edt_devname, int unit, int channel)
    int  pdv_set_timeout(PdvDev pdv_p, int value)
 #   int  pdv_setsize(PdvDev pdv_p, int width, int height)
    int  pdv_multibuf(EdtDev edt_p, int numbufs)
    int pdv_get_shutter_method(PdvDev pdv_p, uint_t *mcl)
    int  pdv_set_shutter_method(PdvDev pdv_p, int method, unsigned int mcl)
    void pdv_start_images(PdvDev pdv_p, int count)
    void pdv_flush_fifo(PdvDev pdv_p)
    int pdv_close(PdvDev pdv_p)
    unsigned char  *pdv_wait_images(PdvDev pdv_p, int count)
    int  pdv_get_height(PdvDev pdv_p)
    int  pdv_get_width(PdvDev pdv_p)
    int  pdv_set_height(PdvDev pdv_p, int)
    int  pdv_set_width(PdvDev pdv_p, int)
    int  pdv_get_depth(PdvDev pdv_p)                
    int  pdv_serial_write(PdvDev ed, const char *buf, int size)
    int  pdv_serial_read(PdvDev fd, char *buf, int size)
    int  pdv_serial_wait(EdtDev pd, int msecs, int count)
    void pdv_serial_reset(PdvDev pdv_p)
    void pdv_serial_set_block_size(int newsize)
    int pdv_serial_set_baud(EdtDev  edt_p, int  baud)

    int         edt_set_max_buffers(EdtDev edt_p, int newmax)
    int         edt_get_max_buffers(EdtDev edt_p)

    uint_t        edt_reg_read(EdtDev edt_p, uint_t desc)
    void          edt_reg_write(EdtDev edt_p, uint_t desc, uint_t val) 
    uint_t        edt_reg_or(EdtDev edt_p, uint_t desc, uint_t val) 
    uint_t        edt_reg_and(EdtDev edt_p, uint_t desc, uint_t val)
    
    int     edt_abort_current_dma(EdtDev edt_p)
    unsigned char  *edt_check_for_buffers(EdtDev edt_p, uint_t count)
    int             edt_user_dma_wakeup(EdtDev edt_p)
    int             edt_ring_buffer_overrun(EdtDev edt_p)
    EdtDmaWaitStatus          edt_get_wait_status(EdtDev edt_p)
    pass

cdef class EDTLowLevel:
    cdef PdvDev pdv_p
    cdef object snde_channel #spatial nde2 channel object

    def __cinit__(self):
        self.pdv_p=NULL
        pass
    pass

class EDTSerial:
    parent=None
    def __init__(self,parent):
        self.parent=parent
        pass

    def read(self,nbytes=1):
        cdef char* buf
        cdef int ret
        cdef object retbytes
        cdef EDTLowLevel lowlevel=self.parent._lowlevel
        
        availsize=pdv_serial_wait(lowlevel.pdv_p,0,nbytes) # wait for the response from the camera, or timeout per EDT camera_config
        if availsize < nbytes:
            sys.stderr.write(f"EDTSerial: read timeout ({nbytes:d} bytes requested; {availsize:d} bytes available)\n")
            pass

        buf = <char*> malloc(availsize+1)
        ret = pdv_serial_read(lowlevel.pdv_p,buf,availsize)
        assert(ret==availsize)
        buf[availsize]=0
        retbytes = PyBytes_FromStringAndSize(buf,availsize)
        free(buf)
        return retbytes

    def write(self,to_write):
        cdef Py_buffer write_buffer
        cdef char* write_ptr
        cdef EDTLowLevel lowlevel=self.parent._lowlevel
        
        PyObject_GetBuffer(to_write,&write_buffer,PyBUF_SIMPLE | PyBUF_ANY_CONTIGUOUS)
        try:
            write_ptr = <char*> write_buffer.buf
            ret=pdv_serial_write(lowlevel.pdv_p,write_ptr,write_buffer.len)
            if ret<0:
                raise IOError(f"Error return {ret:d} from pdv_serial_write")
            pass
        finally:
            PyBuffer_Release(&write_buffer)
            pass

        return ret
        
    pass
        
    

class EDTCapture(metaclass=dgpy.Module):
    recdb=None #spatial NDE2 recording database
    EdtDevName=None #string giving Edt device to open
    EdtUnit=None #integer unit number
    EdtChannel=None # integer channel number from card

    ChannelName=None #spatial NDE2 channel for images

    _width=None #Integer width
    _height=None #Integer height
    _numbufs=None #Number of image buffers

    _FlipLR=None #Boolean flip left right
    _FlipUD=None # Boolean flip up down
    _calcsync=None #True to sync with calculation, use pdv_start_images(1) for
                        # image-by-image mode.
		        # False says we use pdv_start_images(0) to enter free run mode
    _timeoutms=None # capture timeout, in ms. -1 lets the EDT library use a default; 0 means wait forever
    _discardtopline=None #Boolean

    _State=None #"WAITFORACQ","DATACONV", "GETMETADATA","WAITFORMATH","ACQDISABLED" assigned by acquisition thread, locked by _AcqControlLock, _AcqStateCond is notified when this changes

    _Command=None  # "ACQUIRE", "ABORT", "EXIT" assigned by module thread, locked by _AcqControlLock, _AcqCommandCond is notified when this changes

    _AcqControlLock=None #locks _State _Command and associated condition vars
    _AcqStateCond=None #Notified when _State changes
    _AcqCommandCond=None #Notified when _Command changes
    _AcqThread=None #Acquisition thread
    _previous_globalrev_complete_waiter = None # Used for _calcsync mode; set only by sub thread with AcqControlLock locked but used by main thread
    
    _lowlevel=None
    _dynamic_metadata=None #dgpy DynamicMetadata object

    def __init__(self,module_name,recdb,EdtDevName,EdtUnit, EdtChannel,ChannelName,width,height,numbufs=10,FlipLR=False,FlipUD=False,calcsync=True,timeoutms=0,discardtopline=False):
        cdef EDTLowLevel lowlevel=EDTLowLevel()
        self.module_name=module_name
        self.recdb=recdb
        self.EdtDevName=EdtDevName
        self.EdtUnit=EdtUnit
        self.EdtChannel=EdtChannel
        self.ChannelName=ChannelName
        self._width=width
        self._height=height
        self._numbufs=numbufs
        self._FlipLR=FlipLR
        self._FlipUD=FlipUD
        self._calcsync=calcsync
        self._timeoutms=timeoutms
        self._discardtopline=discardtopline

        self._lowlevel=lowlevel
        lowlevel.pdv_p=pdv_open_channel(self.EdtDevName,self.EdtUnit, self.EdtChannel)
        if not lowlevel.pdv_p:
            raise IOError(f"{module_name:s}: pdv_open_channel failed.\nMake sure that the 'edtinit' routine ran during system boot and \n also that you have run 'initcam', e.g. 'initcam -f camera_config/sc6000.cfg'\n")
        self._configure()

        
        self._dynamic_metadata = DynamicMetadata(module_name)
        
        # Transaction required to add a channel
        transact = recdb.start_transaction()

        self._lowlevel.snde_channel = recdb.reserve_channel(transact,snde.channelconfig(ChannelName,module_name,False)) 
        
        transact.end_transaction().globalrev_available()

        self._AcqControlLock=threading.Lock()
        self._AcqStateCond=threading.Condition(self._AcqControlLock)
        self._AcqCommandCond=threading.Condition(self._AcqControlLock)

        self._State="WAITFORACQ"
        self._Command="ACQUIRE"
        
        self._AcqThread = threading.Thread(target=self._AcqThreadCode)
        self._AcqThread.start() # Won't actually be able to record a transaction until this one ends.
        
    
        atexit.register(self._atexit) # Register an atexit function so that we can cleanly trigger our subthread to end. Otherwise we might well crash on exit.
        pass

    def _configure(self):
        # call with capture aborted
        cdef EDTLowLevel lowlevel=self._lowlevel
        pdv_set_timeout (lowlevel.pdv_p,self._timeoutms)
        if self.discardtopline:
            pdv_set_width(lowlevel.pdv_p,self._width)
            pdv_set_height(lowlevel.pdv_p,self._height+1)
            pass
        else:
            pdv_set_width(lowlevel.pdv_p,self._width)
            pdv_set_height(lowlevel.pdv_p,self._height)
            pass

        maxbuf=edt_get_max_buffers(lowlevel.pdv_p)
        if (maxbuf < self._numbufs):
            edt_set_max_buffers(lowlevel.pdv_p,self._numbufs)
            pass

        pdv_multibuf(lowlevel.pdv_p,self._numbufs)
        pass
    
    def _atexit(self):
        #print("Edt: Performing atexit()")
        with self._capture_running_cond:
            self._capture_exit = True;
            self._stop_temporarily()
            pass

        self.AcqThread.join()
        
        pass

    def _AbortImages(self):
        cdef EDTLowLevel lowlevel=self._lowlevel
        pdv_start_images(lowlevel.pdv_p,1);  # get out of free run mode
        edt_abort_current_dma(lowlevel.pdv_p); # abort dma
        while (edt_check_for_buffers(lowlevel.pdv_p,1)):
            # empty ring buffer
            pass
        edt_abort_current_dma(lowlevel.pdv_p); # abort dma 
        pdv_flush_fifo(lowlevel.pdv_p); # flush fifo
        pass
         

    def _AbortAcq(self):
        with self._AcqCaptureLock:
            self._Command="ABORT"
            self._AbortImages()
            self._AcqCommandCond.notify()
            self._AcqStateCond.wait()
            pass
        pass

    def _RestartAcq(self):
        with self._AcqCaptureLock:
            self._Command="ACQUIRE"
            self._AcqCommandCond.notify()
            self._AcqStateCond.wait()
            pass
        pass

    @property
    def width(self):
        return self._width

    @width.setter
    def width(self,width):
        self._AbortAcq()
        self._width=int(width)
        self._configure()
        self._RestartAcq()
        pass

    
    @property
    def height(self):
        return self._height

    @height.setter
    def height(self,height):
        self._AbortAcq()
        self._height=int(height)
        self._configure()
        self._RestartAcq()
        pass

        
    @property
    def numbufs(self):
        return self._numbufs

    @numbufs.setter
    def numbufs(self,numbufs):
        self._AbortAcq()
        self._numbufs=int(numbufs)
        self._configure()
        self._RestartAcq()
        pass

    @property
    def FlipLR(self):
        return self._FlipLR

    @FlipLR.setter
    def FlipLR(self,FlipLR):
        self._AbortAcq()
        self._FlipLR=bool(FlipLR)
        self._configure()
        self._RestartAcq()
        pass
    
    @property
    def FlipUD(self):
        return self._FlipUD

    @FlipUD.setter
    def FlipUD(self,FlipUD):
        self._AbortAcq()
        self._FlipUD=bool(FlipUD)
        self._configure()
        self._RestartAcq()
        pass

    @property
    def calcsync(self):
        return self._calcsync

    @calcsync.setter
    def calcsync(self,calcsync):
        self._AbortAcq()
        self._calcsync=bool(calcsync)
        self._configure()
        self._RestartAcq()
        pass

    @property
    def timeoutms(self):
        return self._timeoutms

    @timeoutms.setter
    def timeoutms(self,timeoutms):
        self._AbortAcq()
        self._timeoutms=int(timeoutms)
        self._configure()
        self._RestartAcq()
        pass

    @property
    def discardtopline(self):
        return self._discardtopline

    @discardtopline.setter
    def discardtopline(self,discardtopline):
        self._AbortAcq()
        self._discardtopline=bool(discardtopline)
        self._configure()
        self._RestartAcq()
        pass
    
    def _AcqThreadCode(self):
        cdef EDTLowLevel lowlevel=self._lowlevel
        cdef unsigned char* gotbuf
        cdef EdtDmaWaitStatus waitstatus
        InitCompatibleThread(self,"_edt_capture_thread")

        images_started=False
        
        while True:
            with self._AcqCaptureLock:
                if self._Command=="ABORT":
                    self._State="ACQDISABLED"
                    self._AcqStateCond.notify()
                    self._AcqThreadAbortImages()
                   
                    images_started=False
                    pass
                
                self._AcqCommandCond.wait_for(lambda: self._Command=="ACQUIRE" or self._Command=="EXIT")

                if self._Command=="EXIT":
                    break

                assert(self._Command=="ACQUIRE")
                self._State="WAITFORACQ"
                self._AcqStateCond.notify()
                previous_globalrev_complete_waiter = self._previous_globalrev_complete_waiter
                pass
           
            if previous_globalrev_complete_waiter is not None:
                # If we need to wait for the previous globalrev to be complete
                # before starting a new acquisition, wait here.
                # This can be interrupted from the other thread
                # by calling previous_globalrev_complete_waiter.interrupt()
                interrupted = previous_globalrev_complete_waiter.wait_interruptable()
                aborted = False
                with self._AcqCaptureLock:
                    if not interrupted:
                        # waiter satisfied
                        self._previous_globalrev_complete_waiter = None
                        pass
                                        
                    # Check for stop request
                    if self._Command=="ABORT":
                        continue
                    pass
                if interrupted:
                    continue  # So we can wait again if we were interrupted and somehow didn't have a stop request
                pass

            if edt_ring_buffer_overrun(lowlevel.pdv_p):
                sys.stderr.write(f"{self.module_name:s}: EDTCAPTURE WARNING: RING BUFFER OVERRUN\n")
                pass
                
            if not self._calcsync and not images_started:
                if EDT_ENABLE_FAST_REARM:
                    # must turn FAST_REARM on for stream acquistion, 
                    # because we had to force it OFF for frame-by-frame mode 
                    edt_reg_or(lowlevel.pdv_p,PDV_CL_CFG2,PDV_CL_CFG2_FAST_REARM); # 0x40
                    pass
                    
                pdv_start_images(lowlevel.pdv_p,0) #reenable streaming
                pass
            elif self._calcsync:
                if EDT_ENABLE_FAST_REARM:
                    #must turn FAST_REARM off for frame-by-frame acquistion, 
                    #lest the board auto rearm when it shouldn't 
                    edt_reg_and(lowlevel.pdv_p,PDV_CL_CFG2,~PDV_CL_CFG2_FAST_REARM); # turn off 0x40
                    pass
                pdv_start_images(lowlevel.pdv_p,1) #request single image
                pass

            with self._AcqCaptureLock:
                if self._Command!="ACQUIRE":
                    continue # go back to main loop
                    
            #wait for an image
            gotbuf= pdv_wait_images(lowlevel.pdv_p,1)
            if edt_ring_buffer_overrun(lowlevel.pdv_p):
                sys.stderr.write(f"{self.module_name:s}: EDTCAPTURE WARNING: RING BUFFER OVERRUN\n")
                pass
            if edt_reg_read(lowlevel.pdv_p,PDV_STAT) & PDV_OVERRUN:
                sys.stderr.write(f"{self.module_name:s}: EDTCAPTURE WARNING:FIFO OVERRUN\n")
                pass

            waitstatus=edt_get_wait_status(lowlevel.pdv_p)
            if (waitstatus==EDT_WAIT_OK) :
                ImgWidth=pdv_get_width(lowlevel.pdv_p)
                ImgHeight=pdv_get_height(lowlevel.pdv_p)
                if self._discardtopline:
                    ImgHeight-=1
                    pass
                ImgDepth=pdv_get_depth(lowlevel.pdv_p)
                pass
            #elif waitstatus==EDT_WAIT_USER_WAKEUP:
            #    continue #switch to abort mode at request of other thread
            elif waitstatus==EDT_WAIT_TIMEOUT:
                self._AbortImages()
                images_started=False
                continue
            else:
                sys.stderr.write(f"{self.module_name:s}: EDT Wait status was {waitstatus:d} (see EDT_WAIT_... in libedt.h)\n")
                continue

            transact = self.recdb.start_transaction()
            if ImgDepth <= 8:
                image_recording_ref = snde.create_ndarray_ref(transact,self.result_depth_channel_ptr,self,snde.SNDE_RTN_INT8)
                bytes_per_pixel=1
                pass
            else:
                assert(ImgDepth <= 16)
                image_recording_ref = snde.create_ndarray_ref(transact,self.result_depth_channel_ptr,self,snde.SNDE_RTN_INT16)
                bytes_per_pixel=2
                pass
            # indicate that we are going to request dynamic meta data
            image_recording_ref.rec.recording_needs_dynamic_metadata()

            # end the transaction
            transobj = transact.run_in_background_and_end_transaction(self.dynamic_metadata.Snapshot().Acquire,(image_recording_ref.rec,))

            image_recording_ref.allocate_storage([ImgWidth,ImgHeight],True) # Fortran mode

            image_recording_data=image_recording_ref.data #numpy array to store data into

            convert_edt_image(gotbuf,<unsigned char*>image_recording_data.data,bytes_per_pixel, ImgWidth, ImgHeight,<int>self._discardtopline,<int>self.FlipLR, <int>self.FlipUD)

            metadata=snde.constructible_metadata()
            metadata.AddMetaDatum(snde.metadatum("edt_bit_depth",ImgDepth))
            image_recording_ref.rec.mark_metadata_done() #mark nondynamic meta data as complete
            image_recording_ref.rec.mark_data_ready()
            
            #image has been acquired into gotbuf and converted; 
            
            pass
        pass

    def open_serial(self,baud):
        cdef EDTLowLevel lowlevel=self._lowlevel
        pdv_serial_reset(lowlevel.pdv_p)
        pdv_serial_set_block_size(1)
        pdv_serial_set_baud(lowlevel.pdv_p,baud)
        return EDTSerial(self)
    
            
    pass
    
