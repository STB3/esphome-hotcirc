# HotCirc – PCB Manufacturing Files / Fertigungsdaten Leiterplatte

[English](#english) | [Deutsch](#deutsch)

---

## English

This directory contains the manufacturing files for the HotCirc PCB (Gerber, drill data and bill of materials), exported from KiCad.

Export date: 2026-11-04

### Overview

The board is **two-layer** (top and bottom copper, no inner layers), made from standard FR4 with a thickness of approx. **1.6 mm**.

| File | Type | Content |
|---|---|---|
| `HotCirc-F_Cu.gbr` | Gerber | Copper, top (front) |
| `HotCirc-B_Cu.gbr` | Gerber | Copper, bottom (back) |
| `HotCirc-F_Mask.gbr` | Gerber | Solder mask, top |
| `HotCirc-B_Mask.gbr` | Gerber | Solder mask, bottom |
| `HotCirc-F_Paste.gbr` | Gerber | Solder paste stencil, top (SMD pads) |
| `HotCirc-B_Paste.gbr` | Gerber | Solder paste stencil, bottom |
| `HotCirc-F_Silkscreen.gbr` | Gerber | Silkscreen / component markings, top |
| `HotCirc-B_Silkscreen.gbr` | Gerber | Silkscreen / component markings, bottom |
| `HotCirc-Edge_Cuts.gbr` | Gerber | Board outline (routing contour) |
| `HotCirc-PTH.drl` | Excellon | Plated through holes (vias, THT pads) |
| `HotCirc-NPTH.drl` | Excellon | Non-plated holes (e.g. mounting holes) |
| `HotCirc_BOM.html` | HTML | Interactive bill of materials (parts, values, placement) |

### Layer stack

```
  F_Silkscreen   silkscreen top
  F_Paste        solder paste top
  F_Mask         solder mask top
  F_Cu           copper top
  ── substrate (FR4, approx. 1.6 mm) ──
  B_Cu           copper bottom
  B_Mask         solder mask bottom
  B_Paste        solder paste bottom
  B_Silkscreen   silkscreen bottom
```

### Manufacturing notes

- **PCB fab:** Pack all `.gbr` and `.drl` files into one ZIP archive for ordering. The BOM is not needed for bare-board fabrication.
- **Drill files:** Plated (PTH) and non-plated (NPTH) holes are supplied as separate files. Most fabs (JLCPCB, PCBWay, Aisler etc.) detect both automatically.
- **Bottom side:** The bottom paste layer (`B_Paste`) is practically empty – there are virtually no SMD parts on the bottom side. A stencil is only needed for the top side.
- **Assembly:** Open `HotCirc_BOM.html` in a web browser. It groups parts by value and highlights their position on the board – handy for hand assembly.

### Check before ordering

Review the Gerber files before ordering, e.g. with the KiCad Gerber viewer, [tracespace.io](https://tracespace.io/view/) or your fab's online viewer.

---

## Deutsch

Dieses Verzeichnis enthält die Fertigungsdaten der HotCirc-Leiterplatte (Gerber, Bohrdaten und Stückliste), exportiert aus KiCad.

Stand des Exports: 04.11.2026

### Übersicht

Die Platine ist **zweilagig** (Top- und Bottom-Kupfer, keine Innenlagen) und besteht aus Standard-FR4 mit einer Dicke von ca. **1,6 mm**.

| Datei | Typ | Inhalt |
|---|---|---|
| `HotCirc-F_Cu.gbr` | Gerber | Kupferlage oben (Top / Front) |
| `HotCirc-B_Cu.gbr` | Gerber | Kupferlage unten (Bottom / Back) |
| `HotCirc-F_Mask.gbr` | Gerber | Lötstoppmaske oben |
| `HotCirc-B_Mask.gbr` | Gerber | Lötstoppmaske unten |
| `HotCirc-F_Paste.gbr` | Gerber | Lotpastenschablone oben (SMD-Pads) |
| `HotCirc-B_Paste.gbr` | Gerber | Lotpastenschablone unten |
| `HotCirc-F_Silkscreen.gbr` | Gerber | Bestückungsdruck / Beschriftung oben |
| `HotCirc-B_Silkscreen.gbr` | Gerber | Bestückungsdruck / Beschriftung unten |
| `HotCirc-Edge_Cuts.gbr` | Gerber | Platinenkontur (Fräsumriss) |
| `HotCirc-PTH.drl` | Excellon | Bohrungen, durchkontaktiert (Vias, THT-Pads) |
| `HotCirc-NPTH.drl` | Excellon | Bohrungen, nicht durchkontaktiert (z. B. Befestigungslöcher) |
| `HotCirc_BOM.html` | HTML | Interaktive Stückliste (Bauteile, Werte, Positionen) |

### Lagenaufbau

```
  F_Silkscreen   Bestückungsdruck oben
  F_Paste        Lotpaste oben
  F_Mask         Lötstopp oben
  F_Cu           Kupfer oben
  ── Basismaterial (FR4, ca. 1,6 mm) ──
  B_Cu           Kupfer unten
  B_Mask         Lötstopp unten
  B_Paste        Lotpaste unten
  B_Silkscreen   Bestückungsdruck unten
```

### Hinweise zur Fertigung

- **Leiterplattenhersteller:** Für die Bestellung alle `.gbr`- und `.drl`-Dateien in ein ZIP-Archiv packen. Die BOM wird für die reine Platinenfertigung nicht benötigt.
- **Bohrdaten:** PTH und NPTH liegen als getrennte Dateien vor. Die meisten Hersteller (JLCPCB, PCBWay, Aisler u. a.) erkennen beide automatisch.
- **Unterseite:** Die Pastenlage unten (`B_Paste`) ist nahezu leer – auf der Unterseite befinden sich praktisch keine SMD-Bauteile. Eine Schablone wird nur für die Oberseite benötigt.
- **Bestückung:** `HotCirc_BOM.html` im Browser öffnen. Die Stückliste zeigt Bauteile gruppiert nach Wert und hebt ihre Position auf der Platine hervor – praktisch für die Handbestückung.

### Kontrolle vor der Bestellung

Die Gerber-Daten lassen sich vorab prüfen, z. B. mit dem Gerber-Viewer in KiCad, mit [tracespace.io](https://tracespace.io/view/) oder mit dem Online-Viewer des Herstellers.
