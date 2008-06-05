Contacts:

Daniel Drake <ddrake@brontes3d.com>
Feel free to fire odd questions at me over the summer, I'll try and reply when
I have time.

Jerry Vadenais <jvadenais@mmm.com>
Jerry is the main hardware guy on this project. He selected the Eureka IC,
implemented the interface-level protocol that was designed, etc. He's the
person to inform about hardware bugs, requested design changes, test requests,
etc.

Phil Lamoreaux <palamoreaux@mmm.com>
Phil leads the systems group and will integrate the results of this project
into our dental scanning application.

Justin Bronder <jsbronder@mmm.com>
Justin is "the Gentoo guy" at the company. Jerry has very little Linux
experience, and Phil does have Linux experience but more from a
user/high-level-developer perspective. So Justin is the man for coordinating
testing with Phil and Jerry. I ask him to reset systems that have hung, set up
serial consoles, and things like that. IRC can be a good medium for such
requests, he's jsbronder on freenode.

Ed Tekeian <etekeian@mmm.com>
Ed is the technical director who oversees this project and others.

Chris Sarno <cmsarno@mmm.com>
Chris is the accountant, so payment questions go his way.


Systems:

I use systemslab02.mmm.com as the "main" development box. It is present in
the hardware lab which is where Jerry resides - so when Jerry is running
tests, he's typically using this system. It usually has a test wand hooked up
to it, which is a wand that always sends "ramp images" (gradients) without
needing triggering through the microcontroller. Jerry also has a PCI analyzer
device which allows him to monitor interrupts, memory accesses, register
values, etc. So if there are strange things going on, Jerry will likely ask
for you to run stuff on this system while he keeps an eye on the analyzer.

systemslab01.mmm.com is Phil's development workstation for this project. It
has a real wand (I think) hooked up, so this one does need triggering, and
assuming it is working correctly it will return "real life" images of whatever
is in front of it. Lighting conditions might mean that you don't see much
however.

systemslab01 is connected to lex-mastication.mmm.com with a serial console
cable, useful for debugging.

systemslab02 used to be connected to systemslab01 with a serial console, but
I don't think thats the case any more since it was moved into the hardware
lab. But it could probably be moved and linked up again, if required.

Before using a system it is a good idea to mail Phil for systemslab01 or Jerry
for systemslab02 to check that they are not using it (and they will not
interrupt you).

You can generally do what you want with these systems (i.e. feel free to
recompile the kernel with more debug options if useful) but it's probably
worth telling Justin what you're doing first.


Triggering the wand:

Triggering starts image capture. You need a checkout of our "apps" repository
to do this. I usually just use Phil's checkout on systemslab01:
	sudo su - phil
	<hit enter to make keychain go away>
	cd work/apps
	./startwandtrigger


Development:

I do main development on systemslab02 in my home directory. I use git for
version control and distribution.

Jerry also runs the software from here, so I try to keep it "usable" - i.e.
I do development work in a branch, and when finished I check out the
known-working master branch so that things don't fall over if he starts tests
after.

I sometimes do development work on systemslab01 (i.e. when 02 is in use) but
I pull the changes back to systemslab02 after.

Phil runs driver code out of ~phil/b3dfg on systemslab01. He doesn't know
about git or anything, I manually "sudo su - phil" and then pull the latest
driver code into there when I am ready for him to use a different revision.


Components:

I currently keep all components in the same git repository.

The kernel driver (b3dfg) is in the root directory.

Our scanning software will interface with the driver through a userspace
library, libb3dfg. libb3dfg is in the "libb3dfg" subdirectory, and is quite
simple: it basically just provides nice function wrappers around ioctls.

The interface design documents are written in latex and are present in the
"design" subdirectory. After making changes, I usually send Jerry the latest
revision (PDF format) and a changelog.

This role includes keeping all 3 in sync.


Open sourcing the driver:

The company is open-source friendly and the plan from the start has been
to open source the driver when complete. I share the opinion of most kernel
developers that this is not only a legal requirement, it makes a lot of sense
since more people will look at the code, and anyone who changes internal
kernel API will fix up our driver for us (reducing maintenance on our side).

That said, I am not sure that the driver would be accepted into the upstream
kernel due to us focusing on producing working hardware quickly, rather than
producing perfect hardware with a perfect driver. For example, we allocate
silly-sized frame buffers early on, which might be unused for a lot of time.
Would the kernel developers reject this ugly code even though it'll only
ever be used by us? I'm not sure - one way to find out :)

It may also be useful to publish the driver as GPL even before it is complete,
it's always useful to actually show the code when getting help on kernel
development mailing lists etc. If you want to do this, you should get
permission through Ed.


Kernel requirements:

Our current product revision runs Linux v2.6.18. However it looks almost
definite that we will upgrade to 2.6.24 or later for the product generation
that will include this device.

While it is probably possible, it would be tricky to get this driver running
on 2.6.18 because:
 1. kmalloc() seems to unconditionally reject large requests (768kb is too
    much)
 2. There is no nopfn PF handler mechanism, meaning that we'd have to switch
    to a sequence of remap_pfn_range() calls to map the frame buffers into
	userspace.


Possible todo items:
(I've tried to rank these in decreasing order of importance)

Expose the number of dropped triplets. Probably best to return this in the
poll_buffer or wait_buffer ioctls, but I haven't decided if that's the best
approach. The driver must also be careful not to throw away any information
at the other points when it reads the DMA_STS register (the dropped counter
is reset on read). See the expose-dropped-frames branch for an attempted
implementation of this (it crashes). libb3dfg also needs to expose this.

Cable status interrupt implementation. The design docs were recently updated
to include a mechanism for detecting when the wand is connected and
disconnected, but this functionality is not yet implemented in the driver. I
suggest the driver attempts to keep track of the wand status, and offers an
ioctl to check this. It could also inform the status in the poll_buffer and
wait_buffer ioctls. poll() should maybe return POLLERR or something when the
cable is unplugged. The driver should also keep track of the frequency of
cable status change interrupts and if they are coming in too quickly, it
should disable those interrupts (and perhaps report this situation through
the interfaces somehow). This situation may happen with a bad cable which
appears to be connected and disconnected very frequently. All of the above
needs to be exposed through libb3dfg too.

wait_buffer timeout. I recommended to Phil that he uses the wait_buffer ioctl
rather than poll() when waiting for image data. poll() is slightly more
expensive as it must look at the status of all buffers. However there is one
bit of missing functionality: wait_buffer blocks forever, whereas poll can be
given a timeout. We should add an optional timeout to the wait_buffer ioctl.

Locking review. We won't be using threads or anything initially, but the
driver should be as close to usual kernel quality as possible - if two ioctls
can run at the same time and would cause a race, that should be prevented.

TRP_DROPPED overflow. The accuracy of TRP_DROPPED is likely not of huge
importance to the software - if we've dropped frames we're in trouble, but
we probably don't care how many we've dropped. But there is a chance that
TRP_DROPPED will overflow and will read 0 when the driver reads it (when
actually e.g. 256 frames have been dropped), giving a completely false
indication. This should be presented.

Release and submit as open source.

IOMMU. If you boot with the iommu=merge kernel parameter then the kernel will
merge a scatterlist into a single mapping when using dma_map_sg(). There's
discussion of the scatter-gather issues in the design docs. This would be a
neat solution if we can get it working. See the iommu2 branch for an
almost-working implementation.

Bus master loss. For some reason, we sometimes lose the bus master bit in the
PCI_COMMAND register. DMA obviously fails when we aren't a bus master. The
driver automatically "fixes" this by putting that bit back in place when
bus master loss is detected, but this is still a bit of an unknown problem.
Who is unsetting the bit - the BIOS? Linux? The board? Why? It happens quite
often, but what are the exact conditions that cause it? ...

