
Specifications, manuals, academic papers used, and notes written while
developing the Cute kernel.



##Documentation:

#### 01-ProjectGoals.txt

What all of this is about!

#### CuteNotes.txt

Important technical details, bugs, and experiences discovered while
developing this project. That's the repository's _main_ document.

#### papers/sched/00-README

Study notes for a number of scheduling-related papers.

These notes discuss the origin of multi-level feedback queues
(beginning form the 1962's CTSS system, and moving to the Unix jungle
of SVR2/3, Solaris, and the BSDs), spinlocks (from VAX/VMS), general-
purpose kernel preemption, per-CPU runqueues and data areas (VAX/VMS,
DEC OSF/1, and WinNT), and thread scheduling soft and hard affinity
(by the innovative DEC engineers again).

#### papers/fs/00-README

Study notes for a number of filesystem-related papers.

Primary sources for the classcial Multics and Unix SVR2 filesystems,
BSD's FFS, Microsoft's FAT32 & NTFS, and Linux ext2/ext3 are throughly
discussed.



##Material:

Some of the folders stated below also include their own README files.

#### Architecture/

Old and new specs of Intel and AMD x86(-64) CPUs. Closely related
topics like the x86 'memory consistency model' are also included.

#### Manuals/

Official manuals for our development tools (e.g. gcc, and make).
These official documents are usually more than enough; dumbed-
down resources are disastrous for low-level development.

#### Notes/

Miscellaneous resources and self-written notes

#### Specs/

Specs of the 'hardware<->software' interface used by our kernel;
examples include timers, interrupt controllers, and BIOS tables.

#### papers/

Historical and relatively new research that was needed while
working on this interesting project. Detailed notes I've written
while studying these papers are also included.

<div style='text-align:right; margin-right:5em;'>—Ahmed S. Darwish &lt;darwish.07@gmail.com&gt;</div>
