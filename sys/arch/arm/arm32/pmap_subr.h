
/*
 * The l2_dtable tracks L2_BUCKET_SIZE worth of L1 slots.
 *
 * This is normally 16MB worth L2 page descriptors for any given pmap.
 * Reference counts are maintained for L2 descriptors so they can be
 * freed when empty.
 */
struct l2_bucket {
	pt_entry_t *l2b_kva;		/* KVA of L2 Descriptor Table */
	paddr_t l2b_pa;			/* Physical address of same */
	u_short l2b_l1slot;		/* This L2 table's L1 index */
	u_short l2b_occupancy;		/* How many active descriptors */
};



struct l2_dtable {
	/* The number of L2 page descriptors allocated to this l2_dtable */
	u_int l2_occupancy;

	/* List of L2 page descriptors */
	struct l2_bucket l2_bucket[L2_BUCKET_SIZE];
};


/*
 * Given an L1 table index, calculate the corresponding l2_dtable index
 * and bucket index within the l2_dtable.
 */
#define	L2_BUCKET_XSHIFT	(L2_BUCKET_XLOG2 - L1_S_SHIFT)
#define	L2_BUCKET_XFRAME	(~(vaddr_t)0 << L2_BUCKET_XLOG2)
#define	L2_BUCKET_IDX(l1slot)	((l1slot) >> L2_BUCKET_XSHIFT)
#define	L2_IDX(l1slot)		(L2_BUCKET_IDX(l1slot) >> L2_BUCKET_LOG2)
#define	L2_BUCKET(l1slot)	(L2_BUCKET_IDX(l1slot) & (L2_BUCKET_SIZE - 1))

__CTASSERT(0x100000000ULL == ((uint64_t)L2_SIZE * L2_BUCKET_SIZE * L1_S_SIZE));
__CTASSERT(L2_BUCKET_XFRAME == ~(L2_BUCKET_XSIZE-1));

/*
 * Given a virtual address, this macro returns the
 * virtual address required to drop into the next L2 bucket.
 */
#define	L2_NEXT_BUCKET_VA(va)	(((va) & L2_BUCKET_XFRAME) + L2_BUCKET_XSIZE)

extern vaddr_t pmap_kernel_l2dtable_kva;
extern vaddr_t pmap_kernel_l2ptp_kva;
extern paddr_t pmap_kernel_l2ptp_phys;


//pv_addrqh_t pmap_boot_freeq = SLIST_HEAD_INITIALIZER(&pmap_boot_freeq);
//pv_addr_t kernelpages;
extern pv_addr_t kernel_l1pt;
extern pv_addr_t systempage;





/*
 * L2 allocation.
 */
#define	pmap_alloc_l2_dtable()	  \
	    pool_cache_get(&pmap_l2dtable_cache, PR_NOWAIT)
#define	pmap_free_l2_dtable(l2)	 \
	    pool_cache_put(&pmap_l2dtable_cache, (l2))
#define	pmap_alloc_l2_ptp(pap)	  \
	    ((pt_entry_t *)pool_cache_get_paddr(&pmap_l2ptp_cache,\
	    PR_NOWAIT, (pap)))

void
#if defined(PMAP_INCLUDE_PTE_SYNC) && defined(PMAP_CACHE_VIVT)
pmap_free_l2_ptp(bool need_sync, pt_entry_t *l2, paddr_t pa);
#else
pmap_free_l2_ptp(pt_entry_t *l2, paddr_t pa);
#endif





/*
 * Returns a pointer to the L2 bucket associated with the specified pmap
 * and VA, or NULL if no L2 bucket exists for the address.
 */
static inline struct l2_bucket *
pmap_get_l2_bucket(pmap_t pm, vaddr_t va)
{
	const size_t l1slot = l1pte_index(va);
	struct l2_dtable *l2;
	struct l2_bucket *l2b;

	if ((l2 = pm->pm_l2[L2_IDX(l1slot)]) == NULL ||
	    (l2b = &l2->l2_bucket[L2_BUCKET(l1slot)])->l2b_kva == NULL)
		return NULL;

	return l2b;
}




static inline bool
pmap_is_current(pmap_t pm)
{
	if (pm == pmap_kernel() || curproc->p_vmspace->vm_map.pmap == pm)
		return true;

	return false;
}

bool pmap_is_cached(pmap_t pm);



struct l2_bucket *
	pmap_alloc_l2_bucket(pmap_t, vaddr_t);
void	pmap_free_l2_bucket(pmap_t, struct l2_bucket *, u_int);



void	pmap_impl_bootstrap(void);
void	pmap_impl_bootstrap_l1(void);
void	pmap_impl_bootstrap_pools(void);

void	pmap_impl_init(void);
void	pmap_impl_postinit(void);

void	pmap_impl_set_virtual_space(vaddr_t, vaddr_t);




