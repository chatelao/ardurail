/*
	SRCPMessages - SRCP Meldungen

	Siehe auch: http://srcpd.sourceforge.net/srcp/

	Copyright (c) 2010 Marcel Bernet.  All right reserved.

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <stdio.h>
#include <Arduino.h>
#include "SRCPMessages.h"

namespace srcp
{
/**
 * 	Die Antworten des Servers an den Client im Kommandomodus beginnen immer mit einem Zeitstempel, gefolgt von einem
 *  numerischen Antwortcode und weiteren Angaben. Der numerische Antwortcode ist in Gruppen gegliedert
 *  1xx INFO: Informationen, Ergebnisse
 *	2xx OK: Befehl wurde angenommen und zur Ausfuehrung gebracht
	4xx/5xx ERROR: Eine Fehlerbedingung ist aufgetreten, der Befehl wird ignoriert und nicht ausgefuehrt.
 *	6xx ERROR: Ein serverspezifischer Fehlerzustand ist aufgetreten. Der betreffende Befehl wird nicht ausgefuehrt. Details sind der Serverdokumentation zu entnehmen.
 */

char VERSION[] = "srcpd V1.0; SRCP 0.8.2";
const char INFO[] = "%ld 100 INFO %d %s %d %d";
const char INFO_GL[] = "%ld 100 INFO %d GL %d %s";
const char INFO_SM[] = "%ld 100 INFO %d SM %d GET %d %d";
const char OK[] = "%ld %d OK";
const char OK201[] = "%ld %d OK PROTOCOL SRCP";
const char GO[] = "%ld 200 OK GO %d";
const char ERROR[] = "%ld %d ERROR";

/**
 400 ERROR Protokoll wird nicht unterstuetzt
 401 ERROR Verbindungsmodus wird nicht unterstuetzt.
 402 ERROR Ungenuegende Angaben.
 410 ERROR Unbekanntes Kommando.
 410 ERROR unknown command Unbekanntes Kommando.
 411 ERROR unknown value Unbekannter Wert.
 412 ERROR wrong value Ein Parameter ist ausserhalb des zulaessigen Bereichs.
 413 ERROR temporarily prohibited Befehl ist derzeit verboten.
 414 ERROR device locked Geraet ist gesperrt.
 415 ERROR forbidden Befehl ist grundsaetzlich verboten.
 416 ERROR no data Keine Informationen verfuegbar.
 417 ERROR timeout Ein Zeitlimit ist ueberschritten.
 418 ERROR list to long Die Parameterliste ist zu lang.
 419 ERROR list to short Die Parameterliste ist zu kurz.
 420 ERROR unsupported device protocol Dieses Geraet unterstuetzt das angegebene Protokoll nicht.
 421 ERROR unsupported device Dieses Geraet wird auf dem angegeben Bus nicht unterstuetzt.
 422 ERROR unsupported device group Die Geraetegruppe ist auf dem angegebenen Bus nicht verfuegbar.
 423 ERROR unsupported operation Die angeforderte Aktion/Befehl wird von diesem Geraet nicht unterstuetzt.
 424 ERROR device reinitialized Das betreffende Geraet wurde neu initialisiert. Ein GET Befehl muss zunaechst ausgefuehrt werden.
 499 ERROR unspecified error Ein Fehler ist aufgetreten, kann aber nicht naeher angegeben werden. Diese Fehlermeldung SOLL vermieden werden.
 500 ERROR Keine Ressourcen mehr frei.
 */

const char* SRCPMessages::version()
{
	return	( VERSION );
}
char* SRCPMessages::error(int id)
{
	sprintf(buf, ERROR, millis(), id);
	return (buf);
}

char* SRCPMessages::ok()
{
	sprintf(buf, OK, millis(), 200);
	return (buf);
}
char* SRCPMessages::ok201()
{
	sprintf(buf, OK201, millis(), 201 );
	return (buf);
}
char* SRCPMessages::ok202()
{
	sprintf(buf, OK, millis(), 202 );
	return (buf);
}
char* SRCPMessages::go(int clientId)
{
	sprintf(buf, GO, millis(), clientId);
	return (buf);
}
char* SRCPMessages::info( int port, char* device, int addr, int value )
{
	sprintf(buf, INFO, millis(), port, device, addr, value );
	return (buf);
}
char* SRCPMessages::info( int bus, int addr, int cv, int value )
{
	sprintf(buf, INFO_SM, millis(), bus, addr, cv, value );
	return (buf);
}
char* SRCPMessages::info( int bus, int addr, char* data)
{
	sprintf(buf, INFO_SM, millis(), bus, addr, data );
	return (buf);
}

// Preinstantiate Objects //////////////////////////////////////////////////////

SRCPMessages Messages;
}





