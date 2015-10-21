#ifndef __ASM_PCI_H
#define __ASM_PCI_H
#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm-generic/pci-bridge.h>
#include <asm-generic/pci-dma-compat.h>

#define PCIBIOS_MIN_IO		0x1000
#define PCIBIOS_MIN_MEM		0

/*
 * Set to 1 if the kernel should re-assign all PCI bus numbers
 */
#define pcibios_assign_all_busses() \
	(pci_has_flag(PCI_REASSIGN_ALL_BUS))

/*
 * PCI address space differs from physical memory address space
 */
#define PCI_DMA_BUS_IS_PHYS	(0)

extern int isa_dma_bridge_buggy;

struct pci_sys_data;
struct pci_ops;
struct pci_bus;
struct device;

struct hw_pci {
#ifdef CONFIG_PCI_DOMAINS
	int		domain;
#endif
#ifdef CONFIG_PCI_MSI
	struct msi_controller *msi_ctrl;
#endif
	struct pci_ops	*ops;
	int		nr_controllers;
	void		**private_data;

	int		(*setup)(int nr, struct pci_sys_data *);
	struct pci_bus *(*scan)(int nr, struct pci_sys_data *);
	void		(*preinit)(void);
	void		(*postinit)(void);
	u8		(*swizzle)(struct pci_dev *dev, u8 *pin);
	int		(*map_irq)(const struct pci_dev *dev, u8 slot, u8 pin);
	resource_size_t (*align_resource)(struct pci_dev *dev,
					  const struct resource *res,
					  resource_size_t start,
					  resource_size_t size,
					  resource_size_t align);
	void		(*add_bus)(struct pci_bus *bus);
	void		(*remove_bus)(struct pci_bus *bus);
};

/*
 * Per-controller structure
 */
struct pci_sys_data {
#ifdef CONFIG_PCI_DOMAINS
	int		domain;
#endif
	struct list_head node;
	int		busnr;	    /* primary bus number		*/
	u64		mem_offset; /* bus->cpu memory mapping offset	*/
	unsigned long	io_offset;  /* bus->cpu IO mapping offset	*/
	struct pci_bus	*bus;	    /* PCI bus				*/
	struct list_head resources; /* root bus resources (apertures)	*/
	struct resource io_res;

	char		io_res_name[12];
				/* Bridge swizzling		*/
	u8		(*swizzle)(struct pci_dev *, u8 *);
				/* IRQ mapping			*/
	int		(*map_irq)(const struct pci_dev *, u8, u8);
				/* Resource alignement requirements	*/
	resource_size_t (*align_resource)(struct pci_dev *dev,
					  const struct resource *res,
					  resource_size_t start,
					  resource_size_t size,
					  resource_size_t align);
	void		(*add_bus)(struct pci_bus *bus);
	void		(*remove_bus)(struct pci_bus *bus);
	void		*private_data;	/* platform controller private data */
};

/*
 * Call this with your hw_pci struct to initialise the PCI system.
 */
void pci_common_init_dev(struct device *, struct hw_pci *);

/*
 * Compatibility wrapper for older platforms that do not care about
 * passing the parent device.
 */
static inline void pci_common_init(struct hw_pci *hw)
{
	pci_common_init_dev(NULL, hw);
}

#ifdef CONFIG_PCI
static inline int pci_get_legacy_ide_irq(struct pci_dev *dev, int channel)
{
	/* no legacy IRQ on arm64 */
	return -ENODEV;
}

static inline int pci_proc_domain(struct pci_bus *bus)
{
	return 1;
}
#endif  /* CONFIG_PCI */

#endif  /* __KERNEL__ */
#endif  /* __ASM_PCI_H */
