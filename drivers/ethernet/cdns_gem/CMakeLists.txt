# SPDX-License-Identifier: Apache-2.0

# Cores
zephyr_library_sources_ifdef(CONFIG_ETH_CDNS_GEM_CORE eth_cdns_gem_core.c)

# Platform specific drivers
zephyr_library_sources_ifdef(CONFIG_ETH_XLNX_GEM eth_xlnx_cdns_gem.c)
