# HotCirc Lernmatrix in Home Assistant

Diese Anleitung zeigt die Lernmatrix des **ESPHome HotCirc** Zirkulationscontrollers als Heatmap-Kachel in einem Home-Assistant-Dashboard an. Die Matrix hat 7 Tage × 48 Halbstunden-Slots, die ECO-Schwelle wird in der Kachel markiert.

## Funktionsweise

```
HotCirc (ESP32)                Home Assistant                       Dashboard
Web-Server v3      ──HTTP──▶   REST-Sensor                ──▶       hotcirc-heatmap-card
/text_sensor/...               sensor.hotcirc_lernmatrix
                               State     = ECO-Level
                               Attribut  = Matrix-JSON
```

Ein HA-State darf höchstens 255 Zeichen lang sein. Die Matrix als JSON ist deutlich länger und passt deshalb nicht hinein. Der REST-Sensor holt das JSON vom Web-Server des ESP und legt es in ein **Attribut** ab, für das diese Grenze nicht gilt. Die Kachel liest dieses Attribut aus.

## Voraussetzungen

- Home Assistant mit Zugriff auf den Ordner `/config`, z. B. über das Add-on „File editor" oder „Studio Code Server"
- HotCirc mit aktivem ESPHome-Web-Server (`web_server: version: 3`)
- Eine **feste IP-Adresse** für den HotCirc, z. B. als DHCP-Reservierung in der Fritzbox

## Dateien

```
/config
├── configuration.yaml          ← Packages einbinden
├── packages/
│   └── hotcirc.yaml            ← REST-Sensor + Recorder-Ausschluss
└── www/
    └── hotcirc-heatmap-card.js ← die Kachel
```

## Installation

### 1. Endpunkt des HotCirc prüfen

Ruf im Browser diese Adresse auf:

```
http://<HOTCIRC-IP>/text_sensor/Learning%20Matrix%20JSON
```

Als Antwort muss JSON mit einem Feld `"value"` kommen, das selbst wieder JSON enthält, etwa `"value": "{\"days\":[...],\"eco_level\":120}"`.

Der Pfad besteht aus dem **Namen des Text-Sensors**, so wie er unter `name:` in der ESPHome-YAML steht. Groß- und Kleinschreibung bleiben erhalten, Leerzeichen werden als `%20` geschrieben. Hast du den Sensor umbenannt, passt du den Pfad entsprechend an.

> **Hinweis:** Ältere Anleitungen verwenden die Object-ID, also `/text_sensor/learning_matrix_json`. Dieses Format ist veraltet und wird ab ESPHome 2026.7.0 nicht mehr unterstützt. Auf älteren Firmware-Ständen funktioniert es noch, erzeugt im Gerätelog aber die Warnung `Deprecated URL format`. Verwende deshalb immer den Entitätsnamen.

### 2. Kachel kopieren

Kopiere `hotcirc-heatmap-card.js` nach `/config/www/`.

> **Wichtig:** Existierte der Ordner `www` vorher nicht, liefert Home Assistant ihn erst nach einem **Neustart** unter `/local/` aus.

Zur Kontrolle öffnest du `http://<HA-IP>:8123/local/hotcirc-heatmap-card.js` im Browser. Der Quellcode der Datei muss angezeigt werden.

### 3. Packages in der `configuration.yaml` aktivieren

Falls noch nicht vorhanden, ergänze:

```yaml
homeassistant:
  packages: !include_dir_named packages/
```

Lege anschließend den Ordner `/config/packages/` an.

### 4. Package `packages/hotcirc.yaml` anlegen

```yaml
# HotCirc – Lernmatrix für die Heatmap-Kachel
rest:
  - resource: http://<HOTCIRC-IP>/text_sensor/Learning%20Matrix%20JSON
    scan_interval: 300
    sensor:
      - name: HotCirc Lernmatrix
        value_template: "{{ (value_json.value | from_json).eco_level }}"
        json_attributes:
          - value

# Großes JSON-Attribut nicht in die Datenbank schreiben
recorder:
  exclude:
    entities:
      - sensor.hotcirc_lernmatrix
```

- Ersetze `<HOTCIRC-IP>` durch die echte IP und gegebenenfalls den Pfad aus Schritt 1.
- Schreib Leerzeichen im Pfad als `%20`, nicht als echte Leerzeichen.
- `scan_interval: 300` fragt die Matrix alle 5 Minuten ab. Das reicht, weil sie sich nur langsam ändert.

> **Hinweis:** Steht in der `configuration.yaml` bereits ein Block `recorder:` mit `exclude: entities:`, kommt es zu einem Konflikt. Home Assistant führt Packages nicht tief zusammen. In dem Fall nimmst du den `recorder`-Block aus dem Package heraus und trägst die Entität zentral in der `configuration.yaml` ein.

### 5. Konfiguration prüfen und neu starten

1. Klicke unter **Entwicklerwerkzeuge → YAML** auf **„Konfiguration prüfen"**.
2. Starte Home Assistant **komplett neu**. Das ist nötig, weil `packages`, `recorder` und eine neue REST-Integration nur beim Start eingelesen werden.
3. Prüfe danach unter **Entwicklerwerkzeuge → Zustände**, ob `sensor.hotcirc_lernmatrix` existiert. Der State muss das ECO-Level zeigen, das Attribut `value` das JSON.

### 6. Ressource registrieren

Unter **Einstellungen → Dashboards → ⋮ → Ressourcen → Ressource hinzufügen** trägst du ein:

| Feld | Wert |
|---|---|
| URL | `/local/hotcirc-heatmap-card.js?v=1` |
| Typ | **JavaScript-Modul** |

> Falls der Menüpunkt „Ressourcen" fehlt, schalte im Benutzerprofil den **Erweiterten Modus** ein.

Lade danach die Seite mit **Strg + Umschalt + R** neu.

### 7. Kachel zum Dashboard hinzufügen

Wähle im Dashboard **Bearbeiten → Karte hinzufügen → Manuell** und füge ein:

```yaml
type: custom:hotcirc-heatmap-card
entity: sensor.hotcirc_lernmatrix
attribute: value
```

## Kachel-Optionen

| Option | Standard | Beschreibung |
|---|---|---|
| `entity` | – | Entität mit den Matrixdaten (Pflicht, sofern `day_entities` nicht gesetzt ist) |
| `language` | HA-Sprache | `de` oder `en`. Ohne Angabe gilt die HA-Spracheinstellung (Deutsch bei `de…`, sonst Englisch) |
| `attribute` | `days` | Attribut, das das JSON enthält. Bei diesem Setup `value` |
| `title` | `HotCirc Lernmatrix` / `HotCirc Learning Matrix` | Titel der Kachel |
| `eco_level` | aus JSON, sonst `120` | Feste ECO-Schwelle |
| `eco_entity` | – | Entität mit dem ECO-Level, hat Vorrang vor dem JSON |
| `thresholds` | `[50, 100, 150]` | Farbgrenzen blau / grün / orange / rot |
| `show_legend` | `true` | Legende ein- oder ausblenden |
| `show_stats` | `true` | Statistikzeile ein- oder ausblenden |
| `day_entities` | – | Alternative Datenquelle: 7 Text-Sensoren mit je 48 komma-getrennten Werten |

Beispiel mit allen Optionen:

```yaml
type: custom:hotcirc-heatmap-card
entity: sensor.hotcirc_lernmatrix
attribute: value
title: Warmwasser-Lernmatrix
language: de
thresholds: [40, 80, 120]
show_stats: false
```

Die Kachel übernimmt Farben und Hell/Dunkel-Modus aus dem aktiven HA-Theme. Slots, deren Wert die ECO-Schwelle erreicht und damit die Pumpe auslösen, sind gelb umrandet. Ein Antippen zeigt die Details zu einem Slot.

## Änderungen später übernehmen

| Was geändert | Was tun |
|---|---|
| `hotcirc-heatmap-card.js` | Im Ressourcen-Eintrag `?v=` hochzählen, Seite mit Strg + Umschalt + R neu laden |
| REST-Sensor in `hotcirc.yaml` | Entwicklerwerkzeuge → YAML → **„REST-Entitäten und Benachrichtigungsdienste"** neu laden |
| `recorder`-Block oder neues Package | Home Assistant neu starten |

## Fehlerbehebung

### Gerätelog: `Deprecated URL format: /text_sensor/learning_matrix_json`

In `resource:` steht noch die alte Object-ID-URL. Ersetze sie durch den Entitätsnamen, also `/text_sensor/Learning%20Matrix%20JSON` (siehe Schritt 1), und lade die REST-Entitäten neu. Ab ESPHome 2026.7.0 liefert die alte URL keine Daten mehr.

### `Custom element doesn't exist: hotcirc-heatmap-card`

Die JS-Datei wurde nicht geladen. Prüfe der Reihe nach:

1. Lässt sich `http://<HA-IP>:8123/local/hotcirc-heatmap-card.js` öffnen? Wenn nicht, liegt die Datei nicht in `/config/www/`, oder der Ordner wurde ohne anschließenden Neustart angelegt.
2. Ist die Ressource als **JavaScript-Modul** mit der URL `/local/…` eingetragen, nicht `/config/www/…`? Ist sie nur einmal vorhanden?
3. Liegt es am Browser-Cache? Den Tab schließen und neu öffnen, Strg + Umschalt + R drücken, die Versionsnummer `?v=` erhöhen.
4. Teste in der Browser-Konsole (Strg + Umschalt + J), ob sich das Modul laden lässt:
   ```js
   customElements.get("hotcirc-heatmap-card")          // undefined = nicht geladen
   import("/local/hotcirc-heatmap-card.js?v=99")         // manuell laden, zeigt Fehler an
   ```
   Lädt der manuelle Import, aber die Ressource nicht: Den Eintrag löschen, die URL **von Hand** neu eintippen (versteckte Zeichen am Ende vermeiden) und `?v=` erhöhen.

### `Entität sensor.hotcirc_lernmatrix nicht gefunden` / `Entity … not found`

Der REST-Sensor wurde nicht angelegt.

- Prüfe, ob in `hotcirc.yaml` noch eine Platzhalter-IP steht statt der echten IP des HotCirc.
- Liefert die URL aus `resource:` im Browser JSON? Siehe Schritt 1.
- Suche unter Entwicklerwerkzeuge → Zustände nach `lernmatrix`. Heißt die Entität anders, passt du `entity:` in der Kachel an.
- Unter Einstellungen → System → Protokolle nach `rest` oder `packages` suchen.

### `Keine Matrixdaten gefunden` / `No matrix data found`

Die Entität existiert, enthält aber kein gültiges JSON. Prüfe unter Entwicklerwerkzeuge → Zustände, ob das Attribut `value` gefüllt ist und ob in der Kachel `attribute: value` steht.

### Home Assistant startet sehr langsam

Mit der Kachel hat das nichts zu tun. Unter **Einstellungen → System → Reparaturen → ⋮ → Integrations-Startzeit** siehst du, welche Integration bremst.
