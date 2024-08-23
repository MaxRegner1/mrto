/* 
   3w-xxxx.c -- 3ware Storage Controller device driver for Linux.

   Written By: Adam Radford <aradford@gmail.com>
   Modifications By: Joel Jacobson <linux@3ware.com>
   		     Arnaldo Carvalho de Melo <acme@conectiva.com.br>
                     Brad Strand <linux@3ware.com>

   Copyright (C) 1999-2010 3ware Inc.

   Kernel compatibility By: 	Andre Hedrick <andre@suse.com>
   Non-Copyright (C) 2000	Andre Hedrick <andre@suse.com>
   
   Further tiny build fixes and trivial hoovering    Alan Cox

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License.

   This program is distributed in the hope that it will be useful,           
   but WITHOUT ANY WARRANTY; without even the implied warranty of            
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             
   GNU General Public License for more details.                              

   NO WARRANTY                                                               
   THE PROGRAM IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OR        
   CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED INCLUDING, WITHOUT      
   LIMITATION, ANY WARRANTIES OR CONDITIONS OF TITLE, NON-INFRINGEMENT,      
   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE. Each Recipient is    
   solely responsible for determining the appropriateness of using and       
   distributing the Program and assumes all risks associated with its        
   exercise of rights under this Agreement, including but not limited to     
   the risks and costs of program errors, damage to or loss of data,         
   programs or equipment, and unavailability or interruption of operations.  

   DISCLAIMER OF LIABILITY                                                   
   NEITHER RECIPIENT NOR ANY CONTRIBUTORS SHALL HAVE ANY LIABILITY FOR ANY   
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL        
   DAMAGES (INCLUDING WITHOUT LIMITATION LOST PROFITS), HOWEVER CAUSED AND   
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR     
   TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE    
   USE OR DISTRIBUTION OF THE PROGRAM OR THE EXERCISE OF ANY RIGHTS GRANTED  
   HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES             

   You should have received a copy of the GNU General Public License         
   along with this program; if not, write to the Free Software               
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 

   Bugs/Comments/Suggestions should be mailed to:                            

   aradford@gmail.com


   History
   -------
   0.1.000 -     Initial release.
   0.4.000 -     Added support for Asynchronous Event Notification through
                 ioctls for 3DM.
   1.0.000 -     Added DPO & FUA bit support for WRITE_10 & WRITE_6 cdb
                 to disable drive write-cache before writes.
   1.1.000 -     Fixed performance bug with DPO & FUA not existing for WRITE_6.
   1.2.000 -     Added support for clean shutdown notification/feature table.
   1.02.00.001 - Added support for full command packet posts through ioctls
                 for 3DM.
                 Bug fix so hot spare drives don't show up.
   1.02.00.002 - Fix bug with tw_setfeature() call that caused oops on some
                 systems.
   08/21/00    - release previously allocated resources on failure at
                 tw_allocate_memory (acme)
   1.02.00.003 - Fix tw_interrupt() to report error to scsi layer when
                 controller status is non-zero.
                 Added handling of request_sense opcode.
                 Fix possible null pointer dereference in 
                 tw_reset_device_extension()
   1.02.00.004 - Add support for device id of 3ware 7000 series controllers.
                 Make tw_setfeature() call with interrupts disabled.
                 Register interrupt handler before enabling interrupts.
                 Clear attention interrupt before draining aen queue.
   1.02.00.005 - Allocate bounce buffers and custom queue depth for raid5 for
                 6000 and 5000 series controllers.
                 Reduce polling mdelays causing problems on some systems.
                 Fix use_sg = 1 calculation bug.
                 Check for scsi_register returning NULL.
                 Add aen count to /proc/scsi/3w-xxxx.
                 Remove aen code unit masking in tw_aen_complete().
   1.02.00.006 - Remove unit from printk in tw_scsi_eh_abort(), causing
                 possible oops.
                 Fix possible null pointer dereference in tw_scsi_queue()
                 if done function pointer was invalid.
   1.02.00.007 - Fix possible null pointer dereferences in tw_ioctl().
                 Remove check for invalid done function pointer from
                 tw_scsi_queue().
   1.02.00.008 - Set max sectors per io to TW_MAX_SECTORS in tw_findcards().
                 Add tw_decode_error() for printing readable error messages.
                 Print some useful information on certain aen codes.
                 Add tw_decode_bits() for interpreting status register output.
                 Make scsi_set_pci_device() for kernels >= 2.4.4
                 Fix bug where aen's could be lost before a reset.
                 Re-add spinlocks in tw_scsi_detect().
                 Fix possible null pointer dereference in tw_aen_drain_queue()
                 during initialization.
                 Clear pci parity errors during initialization and during io.
   1.02.00.009 - Remove redundant increment in tw_state_request_start().
                 Add ioctl support for direct ATA command passthru.
                 Add entire aen code string list.
   1.02.00.010 - Cleanup queueing code, fix jbod thoughput.
                 Fix get_param for specific units.
   1.02.00.011 - Fix bug in tw_aen_complete() where aen's could be lost.
                 Fix tw_aen_drain_queue() to display useful info at init.
                 Set tw_host->max_id for 12 port cards.
                 Add ioctl support for raw command packet post from userspace
                 with sglist fragments (parameter and io).
   1.02.00.012 - Fix read capacity to under report by 1 sector to fix get
                 last sector ioctl.
   1.02.00.013 - Fix bug where more AEN codes weren't coming out during
                 driver initialization.
                 Improved handling of PCI aborts.
   1.02.00.014 - Fix bug in tw_findcards() where AEN code could be lost.
                 Increase timeout in tw_aen_drain_queue() to 30 seconds.
   1.02.00.015 - Re-write raw command post with data ioctl method.
                 Remove raid5 bounce buffers for raid5 for 6XXX for kernel 2.5
                 Add tw_map/unmap_scsi_sg/single_data() for kernel 2.5
                 Replace io_request_lock with host_lock for kernel 2.5
                 Set max_cmd_len to 16 for 3dm for kernel 2.5
   1.02.00.016 - Set host->max_sectors back up to 256.
   1.02.00.017 - Modified pci parity error handling/clearing from config space
                 during initialization.
   1.02.00.018 - Better handling of request sense opcode and sense information
                 for failed commands.  Add tw_decode_sense().
                 Replace all mdelay()'s with scsi_sleep().
   1.02.00.019 - Revert mdelay's and scsi_sleep's, this caused problems on
                 some SMP systems.
   1.02.00.020 - Add pci_set_dma_mask(), rewrite kmalloc()/virt_to_bus() to
                 pci_alloc/free_consistent().
                 Better alignment checking in tw_allocate_memory().
                 Cleanup tw_initialize_device_extension().
   1.02.00.021 - Bump cmd_per_lun in SHT to 255 for better jbod performance.
                 Improve handling of errors in tw_interrupt().
                 Add handling/clearing of controller queue error.
                 Empty stale responses before draining aen queue.
                 Fix tw_scsi_eh_abort() to not reset on every io abort.
                 Set can_queue in SHT to 255 to prevent hang from AEN.
   1.02.00.022 - Fix possible null pointer dereference in tw_scsi_release().
   1.02.00.023 - Fix bug in tw_aen_drain_queue() where unit # was always zero.
   1.02.00.024 - Add severity levels to AEN strings.
   1.02.00.025 - Fix command interrupt spurious error messages.
                 Fix bug in raw command post with data ioctl method.
                 Fix bug where rollcall sometimes failed with cable errors.
                 Print unit # on all command timeouts.
   1.02.00.026 - Fix possible infinite retry bug with power glitch induced
                 drive timeouts.
                 Cleanup some AEN severity levels.
   1.02.00.027 - Add drive not supported AEN code for SATA controllers.
                 Remove spurious unknown ioctl error message.
   1.02.00.028 - Fix bug where multiple controllers with no units were the
                 same card number.
                 Fix bug where cards were being shut down more than once.
   1.02.00.029 - Add missing pci_free_consistent() in tw_allocate_memory().
                 Replace pci_map_single() with pci_map_page() for highmem.
                 Check for tw_setfeature() failure.
   1.02.00.030 - Make driver 64-bit clean.
   1.02.00.031 - Cleanup polling timeouts/routines in several places.
                 Add support for mode sense opcode.
                 Add support for cache mode page.
                 Add support for synchronize cache opcode.
   1.02.00.032 - Fix small multicard rollcall bug.
                 Make driver stay loaded with no units for hot add/swap.
                 Add support for "twe" character device for ioctls.
                 Clean up request_id queueing code.
                 Fix tw_scsi_queue() spinlocks.
   1.02.00.033 - Fix tw_aen_complete() to not queue 'queue empty' AEN's.
                 Initialize queues correctly when loading with no valid units.
   1.02.00.034 - Fix tw_decode_bits() to handle multiple errors.
                 Add support for user configurable cmd_per_lun.
                 Add support for sht->slave_configure().
   1.02.00.035 - Improve tw_allocate_memory() memory allocation.
                 Fix tw_chrdev_ioctl() to sleep correctly.
   1.02.00.036 - Increase character ioctl timeout to 60 seconds.
   1.02.00.037 - Fix tw_ioctl() to handle all non-data ATA passthru cmds
                 for 'smartmontools' support.
   1.26.00.038 - Roll driver minor version to 26 to denote kernel 2.6.
                 Add support for cmds_per_lun module parameter.
   1.26.00.039 - Fix bug in tw_chrdev_ioctl() polling code.
                 Fix data_buffer_length usage in tw_chrdev_ioctl().
                 Update contact information.
   1.26.02.000 - Convert driver to pci_driver format.
   1.26.02.001 - Increase max ioctl buffer size to 512 sectors.
                 Make tw_scsi_queue() return 0 for 'Unknown scsi opcode'.
                 Fix tw_remove() to free irq handler/unregister_chrdev()
                 before shutting down card.
                 Change to new 'change_queue_depth' api.
                 Fix 'handled=1' ISR usage, remove bogus IRQ check.
   1.26.02.002 - Free irq handler in __tw_shutdown().
                 Turn on RCD bit for caching mode page.
                 Serialize reset code.
   1.26.02.003 - Force 60 second timeout default.
*/

#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/typ   /interrupt.h>
#include9VcBwoformation.
   1.26.02.000 - Convert driver to pci_driver format.
 iver format.
 iver format.
 iver format.
 iver format.
 iver format.
 iver format.
 iver format.
 iver format.
 iver format.
 iver format.
 iver format.
 iver fVcBx strings.
   1.02.00.025 - Fix command interrupt spurious error messages.
ormat.
 iver fVcBx strings.
   1.02.00.025 - Fix command interrupt spurious error messages.
ormat.
 iver fVcBx strings.
   1.02.00.025 - Fix command interrupt spurious error messages.
ormat.
 iver fVcBx strings.
   1.02.00.09VcB - Fix command interrupt spurious error messages.
ormat.
 iver f for raw command packet post from userspace
                 with sglist fragments (parameter and io).
   1.02.00.012 - Fix read cN 512 sectors.
                 Make tw_scsi_queue() cBeue() cBeue() cBeue() cBeue() cBeue() cBeue() cBeue() cBeue() cBTK8h Re Make tw_scsi_queue() cBeue() cBeue() cBeue() cBeue() cBeue() cBeue() cBeue() cBeue() cBTK8h Re Make tw_scsi_queue() cBeue() cBeue() cBeue() cBeue() cBeue() cBeue() cBeue() cBeue() cBTK8h Re Make tw_scsi_queue() cBeue() cBeu ke 8ha
 iver format.
 iver format.
 iver format.
 iver format.
 iver fors error messages.
ormat.
 iver fVcBx strings.
   1.02.00.025 - Fix2 cBeumessages.
ormat.
8ormat.
8ormat.
8ormat.
8ormat.
8ormat.
8ormat.
8ormat.
8ormat.
8Tl