/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2026  PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 *
 *  PCSX2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with PCSX2.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

/* The network adapter's Ethernet cable, as the frontend's link bus: the hooks
 * the rest of the machine calls. The adapter itself is link_adapter.h. */

#pragma once

/* What crosses the bus, named the same in every core that speaks it -- Dolphin's
 * broadband adapter included, so a GameCube and a PS2 can share one hub and
 * simply see each other's broadcasts, as they would on a real one. One message
 * is one Ethernet II frame exactly as it is on the wire, destination MAC first,
 * with neither preamble nor FCS. Every frame goes to every console on the bus,
 * as a hub repeats it; each adapter keeps what is addressed to it. */
#define ETH_LINK_PROTOCOL "ethernet"

/* The link port the cable attaches on. The i.LINK cable has port 0. */
#define ETH_LINK_PORT 1

struct retro_link_interface;
/* Join the frontend's link bus, which is the Ethernet cable. Called before the
 * machine runs, so the MAC the game reads from the adapter's EEPROM already
 * belongs to this console alone. Safe without a frontend bus. */
void ETHlinkAttach(const struct retro_link_interface* link, unsigned port);
void ETHlinkDetach(void);

/* The IOP event that paces the cable. */
void ethInterrupt(void);
/* The IOP has been reset, and every pending IOP event with it. */
void ETHiopReset(void);
/* A vsync: where this console's frame falls on the cable's clock. */
void ETHvsync(void);
