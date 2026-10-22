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

/* The network adapter with its socket on the frontend's link bus. */

#pragma once

#include "net.h"
#include "link_eth.h"

/* The adapter with its socket on the cable. Frames arrive on the EE thread at
 * the moment of the cable's clock they are stamped for, rather than from the
 * receive thread whenever the host happens to see them, so it takes nothing
 * from recv(). DHCP is answered inside the console, with an address no other
 * console on the cable has, so a game set to configure itself automatically
 * finds a LAN with nothing else on it but the other consoles. */
class LinkAdapter : public NetAdapter
{
public:
	LinkAdapter();
	bool isInitialised() override { return true; }
	bool recv(NetPacket* pkt) override { return false; }
	bool recvDue(NetPacket* pkt) override;
	bool send(NetPacket* pkt) override;
	void reloadSettings() override;

private:
	void InitAddresses();
};
