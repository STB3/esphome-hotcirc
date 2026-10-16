/*
 * HotCirc Heatmap Card – Custom Lovelace Card für Home Assistant
 * Visualisiert die Lernmatrix (7 Tage × 48 Halbstunden-Slots) des
 * ESPHome HotCirc Zirkulationscontrollers inkl. ECO-Schwelle.
 *
 * Datenquellen (eine davon):
 *   entity:        Entität, deren Attribut (Standard "days") oder State die
 *                  Matrix enthält – als Objekt, Array oder JSON-String.
 *                  Akzeptiert {days:[{name,values}], eco_level}, das Array
 *                  direkt, oder die ESPHome-Webserver-Antwort {value:"<json>"}.
 *   day_entities:  Liste von 7 Text-Sensoren, je 48 komma-getrennte Werte.
 *
 * Optional:
 *   language:      "de" oder "en". Ohne Angabe wird die HA-Sprache verwendet
 *                  (Deutsch bei "de…", sonst Englisch).
 *   attribute:     Attributname (Standard "days")
 *   eco_entity:    number/sensor mit aktuellem ECO-Level (überschreibt JSON)
 *   eco_level:     fester ECO-Level, falls weder JSON noch eco_entity ihn liefern
 *   thresholds:    Farbgrenzen, Standard [50, 100, 150]
 *   title:         Kartentitel
 *   show_stats:    Statistikzeile anzeigen (Standard true)
 *   show_legend:   Legende anzeigen (Standard true)
 */

const SLOTS = 48;

const I18N = {
  de: {
    days: ["Mo", "Di", "Mi", "Do", "Fr", "Sa", "So"],
    title: "HotCirc Lernmatrix",
    err_config: "Bitte 'entity' oder 'day_entities' angeben",
    err_day_list: "'day_entities' muss eine Liste sein",
    err_entity: (e) => `Entität ${e} nicht gefunden`,
    err_nodata: "Keine Matrixdaten gefunden",
    triggers: "löst Pumpe aus",
    tap_hint: "Slot antippen für Details",
    avg: "Ø Intensität",
    peak: "Peak",
    active: "Aktive Slots",
    pump_slots: "Pumpen-Slots",
    coverage: "Abdeckung",
  },
  en: {
    days: ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"],
    title: "HotCirc Learning Matrix",
    err_config: "Please specify 'entity' or 'day_entities'",
    err_day_list: "'day_entities' must be a list",
    err_entity: (e) => `Entity ${e} not found`,
    err_nodata: "No matrix data found",
    triggers: "triggers pump",
    tap_hint: "Tap a slot for details",
    avg: "Avg. intensity",
    peak: "Peak",
    active: "Active slots",
    pump_slots: "Pump slots",
    coverage: "Coverage",
  },
};

const pickLang = (cfgLang, hassLang) => {
  const l = String(cfgLang || hassLang || (navigator.language || "en")).toLowerCase();
  return l.startsWith("de") ? "de" : "en";
};

const slotTime = (i) =>
  `${String(Math.floor(i / 2)).padStart(2, "0")}:${i % 2 ? "30" : "00"}`;

const esc = (s) =>
  String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

class HotCircHeatmapCard extends HTMLElement {
  setConfig(config) {
    const t = I18N[pickLang(config && config.language)];
    if (!config || (!config.entity && !config.day_entities)) {
      throw new Error(t.err_config);
    }
    if (config.day_entities && !Array.isArray(config.day_entities)) {
      throw new Error(t.err_day_list);
    }
    if (config.language && !["de", "en"].includes(String(config.language).toLowerCase())) {
      throw new Error("language: 'de' | 'en'");
    }
    this._config = {
      attribute: "days",
      thresholds: [50, 100, 150],
      show_stats: true,
      show_legend: true,
      ...config,
    };
    this._sig = null;
    this._selected = null;
    if (!this.shadowRoot) this.attachShadow({ mode: "open" });
    if (this._hass) this.hass = this._hass;
  }

  get _t() {
    return I18N[pickLang(this._config.language, this._hass && this._hass.language)];
  }

  set hass(hass) {
    this._hass = hass;
    if (!this._config) return;
    const data = this._extract(hass);
    data.lang = pickLang(this._config.language, hass.language);
    const sig = JSON.stringify(data);
    if (sig === this._sig) return; // nur neu zeichnen, wenn sich Daten ändern
    this._sig = sig;
    this._data = data;
    this._render();
  }

  _parse(v) {
    if (v === undefined || v === null) return null;
    if (typeof v === "string") {
      try { return JSON.parse(v); } catch (e) { return null; }
    }
    return v;
  }

  _extract(hass) {
    const c = this._config;
    const t = this._t;
    let days = null;
    let eco = c.eco_level !== undefined ? Number(c.eco_level) : 120;

    if (c.entity) {
      const st = hass.states[c.entity];
      if (!st) return { error: t.err_entity(c.entity) };
      let raw = this._parse(st.attributes[c.attribute]);
      if (!raw) raw = this._parse(st.state);
      if (raw && typeof raw.value === "string") raw = this._parse(raw.value);
      if (Array.isArray(raw)) {
        days = raw;
      } else if (raw && Array.isArray(raw.days)) {
        days = raw.days;
        if (raw.eco_level !== undefined && c.eco_level === undefined) eco = Number(raw.eco_level);
      }
    } else {
      days = c.day_entities.map((e, i) => {
        const ent = typeof e === "string" ? { entity: e } : e;
        const st = hass.states[ent.entity];
        const vals = st
          ? String(st.state).split(/[,;\s]+/).map(Number).filter((n) => !isNaN(n))
          : [];
        return { name: ent.name, values: vals };
      });
    }

    if (c.eco_entity) {
      const s = hass.states[c.eco_entity];
      const n = s ? parseFloat(s.state) : NaN;
      if (!isNaN(n)) eco = n;
    }

    if (!days || !days.length) return { error: t.err_nodata };

    days = days.map((d, i) => {
      const values = Array.isArray(d) ? d : d.values || [];
      const out = [];
      for (let s = 0; s < SLOTS; s++) {
        const v = Number(values[s]);
        out.push(isNaN(v) ? null : v);
      }
      // Bei 7 Tagen immer die lokalisierten Namen verwenden, außer bei
      // explizit gesetzten Namen in day_entities.
      const custom = c.day_entities && d && d.name;
      const name = custom
        ? d.name
        : (days.length === 7 ? t.days[i] : (d && d.name) || t.days[i] || `#${i + 1}`);
      return { name, values: out };
    });

    return { days, eco };
  }

  _color(v) {
    const [t1, t2, t3] = this._config.thresholds;
    if (v === null || v === 0) return "var(--hc-none)";
    if (v < t1) return "var(--hc-low)";
    if (v < t2) return "var(--hc-med)";
    if (v < t3) return "var(--hc-high)";
    return "var(--hc-peak)";
  }

  _render() {
    const c = this._config;
    const t = this._t;
    const title = c.title || t.title;
    const root = this.shadowRoot;
    const d = this._data;

    if (d.error) {
      root.innerHTML = `${this._style()}<ha-card header="${esc(title)}"><div class="err">⚠ ${esc(d.error)}</div></ha-card>`;
      return;
    }

    const { days, eco } = d;
    let total = 0, max = -1, maxDay = "", maxSlot = 0, active = 0, trig = 0, n = 0;
    days.forEach((day) =>
      day.values.forEach((v, i) => {
        if (v === null) return;
        n++;
        total += v;
        if (v > 0) active++;
        if (v >= eco) trig++;
        if (v > max) { max = v; maxDay = day.name; maxSlot = i; }
      })
    );
    const slotsTotal = days.length * SLOTS;

    let hours = "";
    for (let h = 0; h < 24; h += 3) {
      hours += `<div class="hour" style="grid-column: span 6">${String(h).padStart(2, "0")}</div>`;
    }

    let rows = "";
    days.forEach((day, di) => {
      rows += `<div class="day">${esc(day.name)}</div>`;
      day.values.forEach((v, si) => {
        const tr = v !== null && v >= eco;
        const sel = this._selected && this._selected[0] === di && this._selected[1] === si;
        rows += `<div class="cell${tr ? " trig" : ""}${sel ? " sel" : ""}${si % 6 === 0 ? " sep" : ""}" data-d="${di}" data-s="${si}"
          style="background:${this._color(v)}"
          title="${esc(day.name)} ${slotTime(si)}: ${v ?? "–"}${tr ? " ⚡ " + t.triggers : ""}"></div>`;
      });
    });

    let info = t.tap_hint;
    if (this._selected) {
      const [di, si] = this._selected;
      const day = days[di];
      if (day) {
        const v = day.values[si];
        info = `<b>${esc(day.name)} ${slotTime(si)}–${slotTime((si + 1) % SLOTS)}</b>: ${v ?? "–"}${
          v !== null && v >= eco ? ` <span class="bolt">⚡ ${t.triggers}</span>` : ""}`;
      }
    }

    const [t1, t2, t3] = c.thresholds;
    const legend = c.show_legend ? `
      <div class="legend">
        <span><i style="background:var(--hc-none)"></i>0</span>
        <span><i style="background:var(--hc-low)"></i>&lt;${t1}</span>
        <span><i style="background:var(--hc-med)"></i>&lt;${t2}</span>
        <span><i style="background:var(--hc-high)"></i>&lt;${t3}</span>
        <span><i style="background:var(--hc-peak)"></i>≥${t3}</span>
        <span class="ecol"><i class="trigbox"></i>≥ ECO ${eco}</span>
      </div>` : "";

    const stats = c.show_stats ? `
      <div class="stats">
        <div><b>${n ? Math.round(total / n) : 0}</b><span>${t.avg}</span></div>
        <div><b>${max < 0 ? "–" : max}</b><span>${t.peak} ${max < 0 ? "" : esc(maxDay) + " " + slotTime(maxSlot)}</span></div>
        <div><b>${active}</b><span>${t.active}</span></div>
        <div><b class="bolt">⚡ ${trig}</b><span>${t.pump_slots}</span></div>
        <div><b>${Math.round((active / slotsTotal) * 100)}%</b><span>${t.coverage}</span></div>
      </div>` : "";

    root.innerHTML = `${this._style()}
      <ha-card header="${esc(title)}">
        <div class="wrap">
          <div class="grid">
            <div></div>${hours}
            ${rows}
          </div>
          <div class="info">${info}</div>
          ${legend}
          ${stats}
        </div>
      </ha-card>`;

    root.querySelector(".grid").addEventListener("click", (ev) => {
      const cell = ev.target.closest(".cell");
      if (!cell) return;
      const di = Number(cell.dataset.d), si = Number(cell.dataset.s);
      const same = this._selected && this._selected[0] === di && this._selected[1] === si;
      this._selected = same ? null : [di, si];
      this._render();
    });
  }

  _style() {
    return `<style>
      :host {
        --hc-none: var(--secondary-background-color, #2c2c2c);
        --hc-low: #2196f3;
        --hc-med: #4caf50;
        --hc-high: #ff9800;
        --hc-peak: #f44336;
        --hc-trigger: #ffeb3b;
      }
      .wrap { padding: 0 16px 16px; }
      .err { padding: 0 16px 16px; color: var(--error-color, #f44336); }
      .grid {
        display: grid;
        grid-template-columns: 30px repeat(${SLOTS}, minmax(0, 1fr));
        gap: 1px;
        user-select: none;
      }
      .hour {
        font-size: 10px;
        color: var(--secondary-text-color);
        padding-bottom: 2px;
      }
      .day {
        font-size: 11px;
        font-weight: 500;
        color: var(--primary-text-color);
        display: flex; align-items: center;
      }
      .cell {
        aspect-ratio: 1 / 1.6;
        border-radius: 2px;
        cursor: pointer;
        min-width: 0;
      }
      .cell.sep { margin-left: 1px; }
      .cell.trig { box-shadow: inset 0 0 0 1.5px var(--hc-trigger); }
      .cell.sel { outline: 2px solid var(--primary-text-color); outline-offset: 0; z-index: 1; }
      .cell:hover { filter: brightness(1.25); }
      .info {
        margin-top: 8px; min-height: 18px;
        font-size: 13px; color: var(--secondary-text-color);
      }
      .info b { color: var(--primary-text-color); font-weight: 500; }
      .bolt { color: var(--hc-trigger); }
      .legend {
        display: flex; flex-wrap: wrap; gap: 4px 12px;
        margin-top: 8px; font-size: 11px; color: var(--secondary-text-color);
      }
      .legend span { display: inline-flex; align-items: center; gap: 4px; }
      .legend i { width: 10px; height: 10px; border-radius: 2px; display: inline-block; }
      .legend i.trigbox { background: transparent; box-shadow: inset 0 0 0 1.5px var(--hc-trigger); }
      .stats {
        display: grid; grid-template-columns: repeat(5, 1fr); gap: 6px;
        margin-top: 12px; padding-top: 12px;
        border-top: 1px solid var(--divider-color);
        text-align: center;
      }
      .stats b { display: block; font-size: 18px; font-weight: 500; color: var(--primary-color); }
      .stats b.bolt { color: var(--hc-trigger); }
      .stats span { font-size: 10px; color: var(--secondary-text-color); }
      @media (max-width: 450px) {
        .stats { grid-template-columns: repeat(3, 1fr); }
      }
    </style>`;
  }

  getCardSize() {
    return 6;
  }

  static getStubConfig() {
    return { entity: "sensor.hotcirc_lernmatrix", attribute: "value" };
  }
}

customElements.define("hotcirc-heatmap-card", HotCircHeatmapCard);

window.customCards = window.customCards || [];
window.customCards.push({
  type: "hotcirc-heatmap-card",
  name: "HotCirc Heatmap",
  description: "HotCirc learning matrix heatmap with ECO threshold / Lernmatrix mit ECO-Schwelle",
});
