/*	$NetBSD$	*/


#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>

#include <sys/bus.h>
#include <sys/kmem.h>

#include <net/if_ether.h>

#include <dev/acpi/acpivar.h>

#include "thunder_bgx.h"
#include "thunder_bgx_var.h"

int bgx_acpi_init_phy(struct bgx *);

#define	BGX_DEVICE_NAME		"BGX"

static ACPI_STATUS
bgx_acpi_register_phy(ACPI_HANDLE handle, uint32_t level,
    void *context, void **status)
{
	struct bgx *bgx = context;
	ACPI_DEVICE_INFO *devinfo;
	ACPI_STATUS rv;

	if (bgx->lmac_count == MAX_LMAC_PER_BGX)
		return AE_ERROR;

	rv = AcpiGetObjectInfo(handle, &devinfo);
	if (ACPI_FAILURE(rv) || devinfo == NULL)
		return AE_OK;	/* we don't want to stop searching */

	ACPI_INTEGER mac[6];
	rv = acpi_dsd_data(handle, "mac-address", (uint8_t *)mac,
	    sizeof(mac));
	if (ACPI_SUCCESS(rv)) {
		for (size_t i = 0; i < __arraycount(mac); i++)
			bgx->lmac[bgx->lmac_count].mac[i] = mac[i];
	}

	// XXXNH need to create phy_if_dev
// 	bgx->lmac[lmac].phy_if_dev =
// 	    OF_device_from_xref(OF_xref_from_node(mdio));


	/* Assign the LMAC */
	bgx->lmac[bgx->lmac_count].lmacid = bgx->lmac_count;

	aprint_debug_dev(bgx->dev, "mac%d: %s\n", bgx->lmac_count,
	    ether_sprintf(bgx->lmac[bgx->lmac_count].mac));

	bgx->lmac_count++;

	ACPI_FREE(devinfo);

	return AE_OK;
}

static ACPI_STATUS
bgx_acpi_map(ACPI_HANDLE handle, UINT32 level, void *ctx, void **retval)
{
	struct bgx *bgx = ctx;
	char instance[] = BGX_DEVICE_NAME "X";
	char string[8];
	ACPI_STATUS rv;

	ACPI_BUFFER buf;
	buf.Pointer = string;
	buf.Length = sizeof(string);

	snprintf(instance, sizeof(instance), BGX_DEVICE_NAME "%d", bgx->bgx_id);

	rv = AcpiGetName(handle, ACPI_SINGLE_NAME, &buf);
	if (ACPI_FAILURE(rv))
		return AE_OK;

	if (strncmp(buf.Pointer, instance, sizeof(instance) - 1))
		return AE_OK;

	rv = AcpiWalkNamespace(ACPI_TYPE_DEVICE, handle, 1,
	    bgx_acpi_register_phy, NULL, bgx, NULL);

	if (ACPI_FAILURE(rv))
		aprint_error_dev(bgx->dev, "couldn't walk namespace: %s\n",
		    AcpiFormatException(rv));

	return rv;
}

int
bgx_acpi_init_phy(struct bgx *bgx)
{
	ACPI_STATUS rv;

	rv = AcpiGetDevices(NULL, bgx_acpi_map, bgx, NULL);
	if (ACPI_FAILURE(rv))
		return ENXIO;

	return 0;
}
