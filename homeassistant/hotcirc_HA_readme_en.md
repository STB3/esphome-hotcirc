# HotCirc Learning Matrix in Home Assistant

This guide shows the learning matrix of the **ESPHome HotCirc** hot water circulation controller as a heatmap card in a Home Assistant dashboard. The matrix covers 7 days × 48 half-hour slots, and the card highlights the ECO threshold.

## How it works

```
HotCirc (ESP32)                Home Assistant                       Dashboard
Web server v3      ──HTTP──▶   REST sensor                ──▶       hotcirc-heatmap-card
/text_sensor/...               sensor.hotcirc_lernmatrix
                               state      = ECO level
                               attribute  = matrix JSON
```

A Home Assistant state is limited to 255 characters. The matrix JSON is much longer and does not fit. The REST sensor fetches the JSON from the ESP's web server and stores it in an **attribute**, which has no such limit. The card reads that attribute.

## Requirements

- Home Assistant with access to the `/config` folder, e.g. via the "File editor" or "Studio Code Server" add-on
- HotCirc with the ESPHome web server enabled (`web_server: version: 3`)
- A **static IP address** for the HotCirc, e.g. a DHCP reservation in your router

## Files

```
/config
├── configuration.yaml          ← enable packages
├── packages/
│   └── hotcirc.yaml            ← REST sensor + recorder exclusion
└── www/
    └── hotcirc-heatmap-card.js ← the card
```

## Installation

### 1. Check the HotCirc endpoint

Open this address in a browser:

```
http://<HOTCIRC-IP>/text_sensor/learning_matrix_json
```

The response must be JSON with a `"value"` field that itself contains JSON, such as `"value": "{\"days\":[...],\"eco_level\":120}"`.

The path depends on the name of the text sensor in your ESPHome YAML. If the address above does not return JSON, try `/text_sensor/Learning%20Matrix%20JSON` or `/text_sensor/learning%20matrix%20json`. Use whichever variant returns JSON in the browser.

### 2. Copy the card

Copy `hotcirc-heatmap-card.js` to `/config/www/`.

> **Important:** If the `www` folder did not exist before, Home Assistant only serves it under `/local/` after a **restart**.

To verify, open `http://<HA-IP>:8123/local/hotcirc-heatmap-card.js` in a browser. You should see the file's source code.

### 3. Enable packages in `configuration.yaml`

Add the following if it is not already there:

```yaml
homeassistant:
  packages: !include_dir_named packages/
```

Then create the folder `/config/packages/`.

### 4. Create the package `packages/hotcirc.yaml`

```yaml
# HotCirc – learning matrix for the heatmap card
rest:
  - resource: http://<HOTCIRC-IP>/text_sensor/learning_matrix_json
    scan_interval: 300
    sensor:
      - name: HotCirc Lernmatrix
        value_template: "{{ (value_json.value | from_json).eco_level }}"
        json_attributes:
          - value

# Keep the large JSON attribute out of the database
recorder:
  exclude:
    entities:
      - sensor.hotcirc_lernmatrix
```

- Replace `<HOTCIRC-IP>` with the real IP and, if needed, the path from step 1.
- `scan_interval: 300` polls the matrix every 5 minutes. That is enough because the matrix only changes slowly.

> **Note:** If `configuration.yaml` already contains a `recorder:` block with `exclude: entities:`, the two will conflict. Home Assistant does not deep-merge packages. In that case, remove the `recorder` block from the package and add the entity to the central `configuration.yaml` instead.

### 5. Check the configuration and restart

1. Under **Developer tools → YAML**, click **"Check configuration"**.
2. Restart Home Assistant **completely**. This is required because `packages`, `recorder` and a new REST integration are only read at startup.
3. Afterwards, check under **Developer tools → States** that `sensor.hotcirc_lernmatrix` exists. Its state should show the ECO level and its `value` attribute the JSON.

### 6. Register the resource

Under **Settings → Dashboards → ⋮ → Resources → Add resource**, enter:

| Field | Value |
|---|---|
| URL | `/local/hotcirc-heatmap-card.js?v=1` |
| Resource type | **JavaScript module** |

> If the "Resources" menu item is missing, enable **Advanced mode** in your user profile.

Then reload the page with **Ctrl + Shift + R**.

### 7. Add the card to a dashboard

In the dashboard, choose **Edit → Add card → Manual** and paste:

```yaml
type: custom:hotcirc-heatmap-card
entity: sensor.hotcirc_lernmatrix
attribute: value
```

## Card options

| Option | Default | Description |
|---|---|---|
| `entity` | – | Entity holding the matrix data (required unless `day_entities` is set) |
| `language` | HA language | `en` or `de`. If omitted, the Home Assistant language setting is used (German for `de…`, English otherwise) |
| `attribute` | `days` | Attribute containing the JSON. `value` in this setup |
| `title` | `HotCirc Learning Matrix` / `HotCirc Lernmatrix` | Card title |
| `eco_level` | from JSON, otherwise `120` | Fixed ECO threshold |
| `eco_entity` | – | Entity providing the ECO level, takes precedence over the JSON |
| `thresholds` | `[50, 100, 150]` | Color boundaries blue / green / orange / red |
| `show_legend` | `true` | Show or hide the legend |
| `show_stats` | `true` | Show or hide the statistics row |
| `day_entities` | – | Alternative data source: 7 text sensors with 48 comma-separated values each |

Example with options:

```yaml
type: custom:hotcirc-heatmap-card
entity: sensor.hotcirc_lernmatrix
attribute: value
title: Hot Water Learning Matrix
language: en
thresholds: [40, 80, 120]
show_stats: false
```

The card takes its colors and light/dark mode from the active Home Assistant theme. Slots whose value reaches the ECO threshold, and therefore trigger the pump, have a yellow outline. Tap a slot to see its details.

## Applying changes later

| What changed | What to do |
|---|---|
| `hotcirc-heatmap-card.js` | Increase `?v=` in the resource entry, reload with Ctrl + Shift + R |
| REST sensor in `hotcirc.yaml` | Developer tools → YAML → reload **"RESTful entities and notify services"** |
| `recorder` block or a new package | Restart Home Assistant |

## Troubleshooting

### `Custom element doesn't exist: hotcirc-heatmap-card`

The JS file was not loaded. Check in this order:

1. Can you open `http://<HA-IP>:8123/local/hotcirc-heatmap-card.js`? If not, the file is not in `/config/www/`, or the folder was created without a restart afterwards.
2. Is the resource registered as a **JavaScript module** with the URL `/local/…`, not `/config/www/…`? Is it listed only once?
3. Is it the browser cache? Close and reopen the tab, press Ctrl + Shift + R, increase the `?v=` number.
4. Test in the browser console (Ctrl + Shift + J) whether the module loads:
   ```js
   customElements.get("hotcirc-heatmap-card")          // undefined = not loaded
   import("/local/hotcirc-heatmap-card.js?v=99")         // load manually, shows errors
   ```
   If the manual import works but the resource does not: delete the entry, retype the URL **by hand** (to avoid hidden trailing characters) and increase `?v=`.

### `Entity sensor.hotcirc_lernmatrix not found`

The REST sensor was not created.

- Check whether `hotcirc.yaml` still contains a placeholder IP instead of the HotCirc's real IP.
- Does the URL in `resource:` return JSON in the browser? See step 1.
- Search for `lernmatrix` under Developer tools → States. If the entity has a different name, adjust `entity:` in the card.
- Search for `rest` or `packages` under Settings → System → Logs.

### `No matrix data found`

The entity exists but contains no valid JSON. Under Developer tools → States, check that the `value` attribute is filled and that the card config contains `attribute: value`.

### Home Assistant starts very slowly

This is unrelated to the card. Under **Settings → System → Repairs → ⋮ → Integration startup time** you can see which integration is slowing things down.
