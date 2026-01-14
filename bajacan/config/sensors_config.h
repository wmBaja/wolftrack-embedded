// Simple header to check for selected board config and include it
// Do not modify or duplicate

#pragma once

#ifndef BOARD_CONFIG_HEADER
#error "Define BOARD_CONFIG_HEADER (e.g., -DBOARD_CONFIG_HEADER=\\\"board_example.h\\\") to select a board config."
#endif

#include BOARD_CONFIG_HEADER
