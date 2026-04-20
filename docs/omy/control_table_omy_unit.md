<style>
/* Expand content area to maximum width */
.control-table-page {
  max-width: 100% !important;
}

.control-table-page .vp-doc {
  max-width: 100% !important;
}

/* Table styling */
.control-table-page table {
  display: block; /* Make table a block to allow overflow scrolling */
  width: 100%;
  overflow-x: auto; /* Enable horizontal scroll */
  white-space: nowrap; /* Prevent text wrapping */
  border-collapse: collapse;
  margin: 1.5rem 0;
  font-size: 0.9rem;
  background: var(--vp-c-bg);
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
}

.control-table-page table th {
  background: var(--vp-c-bg-soft);
  font-weight: 600;
  padding: 0.75rem 0.5rem;
  text-align: left;
  border-bottom: 2px solid var(--vp-c-divider);
  position: sticky;
  top: 0;
  z-index: 10;
}

.control-table-page table td {
  padding: 0.6rem 0.5rem;
  border-bottom: 1px solid var(--vp-c-divider);
  word-break: break-word;
}

/* RAM region styling - subtle blue tint */
.control-table-page table tr.area-ram {
  background-color: rgba(59, 130, 246, 0.05);
}

.control-table-page table tr.area-ram:hover {
  background-color: rgba(59, 130, 246, 0.12);
}

/* EEPROM/ROM region styling - subtle red tint */
.control-table-page table tr.area-eeprom {
  background-color: rgba(239, 68, 68, 0.05);
}

.control-table-page table tr.area-eeprom:hover {
  background-color: rgba(239, 68, 68, 0.12);
}

/* Fallback */
.control-table-page table tr:hover {
  background: var(--vp-c-bg-soft);
}

.control-table-page table tr:last-child td {
  border-bottom: none;
}

/* Responsive design for smaller screens */
@media (max-width: 768px) {
  .control-table-page table {
    font-size: 0.85rem;
  }

  .control-table-page table th,
  .control-table-page table td {
    padding: 0.5rem 0.3rem;
  }
}

/* Section headers */
.control-table-page h2 {
  margin-top: 3rem;
  margin-bottom: 1.5rem;
  padding-bottom: 0.5rem;
  border-bottom: 2px solid var(--vp-c-divider);
}

.control-table-page h3 {
  margin-top: 2rem;
  margin-bottom: 1rem;
  color: var(--vp-c-brand);
}

/* Legend */
.area-legend {
  display: flex;
  gap: 1.5rem;
  margin: 1rem 0 2rem;
  padding: 1rem;
  background: var(--vp-c-bg-soft);
  border-radius: 8px;
  font-size: 0.9rem;
}

.area-legend-item {
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.area-legend-color {
  width: 20px;
  height: 20px;
  border-radius: 4px;
  border: 1px solid var(--vp-c-divider);
}

.area-legend-color.ram {
  background-color: rgba(59, 130, 246, 0.15);
  border-color: #3b82f6;
}

.area-legend-color.eeprom {
  background-color: rgba(239, 68, 68, 0.15);
  border-color: #ef4444;
}
</style>

<script>
(function() {
  if (typeof window === 'undefined' || typeof document === 'undefined') return;
  
  let updateTimer = null

  function scheduleAddAreaClasses() {
    if (updateTimer) clearTimeout(updateTimer)
    updateTimer = setTimeout(addAreaClasses, 50)
  }

  function addAreaClasses() {
    const tables = document.querySelectorAll('.control-table-page table')
    tables.forEach(table => {
      // Re-apply from scratch so tab re-renders stay consistent.
      table.querySelectorAll('tr').forEach(row => {
        row.classList.remove('area-ram', 'area-eeprom')
      })
      table.querySelectorAll('td').forEach(cell => {
        cell.classList.remove('area-ram', 'area-eeprom')
      })

      const headerCells = table.querySelectorAll('th')
      let areaColIdx = -1
      headerCells.forEach((th, idx) => {
        if (th.textContent.trim().toLowerCase() === 'area') {
          areaColIdx = idx
        }
      })

      const rows = table.querySelectorAll('tr')
      rows.forEach(row => {
        const cells = row.querySelectorAll('td')
        if (areaColIdx >= 0 && cells.length > areaColIdx) {
          const areaCell = cells[areaColIdx]
          const areaText = areaCell.textContent.trim().toUpperCase()

          if (areaText === 'RAM') {
            row.classList.add('area-ram')
            areaCell.classList.add('area-ram')
          } else if (areaText === 'EEPROM' || areaText === 'ROM') {
            row.classList.add('area-eeprom')
            areaCell.classList.add('area-eeprom')
          }
        }
      })
    })
  }

  // Run on page load
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', scheduleAddAreaClasses)
  } else {
    scheduleAddAreaClasses()
  }

  // Also run after navigation (for SPA)
  if (typeof window !== 'undefined' && window.addEventListener) {
    window.addEventListener('load', scheduleAddAreaClasses)
    // For VitePress navigation
    setTimeout(scheduleAddAreaClasses, 100)
    setTimeout(scheduleAddAreaClasses, 300)
    setTimeout(scheduleAddAreaClasses, 600)
  }

  // Re-apply when tabs or markdown content re-render.
  document.addEventListener('click', scheduleAddAreaClasses)
  const observer = new MutationObserver(scheduleAddAreaClasses)
  observer.observe(document.body, { childList: true, subtree: true })
})()
</script>

<div class="control-table-page">

# OMY Unit Control Table

This document summarizes the control tables for both OMY units.  
Shared concepts are explained once below, and unit-specific structures/descriptions are separated by tabs.

## Control Table, Data, and Address

The control table is a collection of multiple data fields for device status and control.  
Users can read specific data from the control table through READ Instruction Packets to understand the device status.  
Users can also control the device by modifying specific data in the control table through WRITE Instruction Packets.  
Address is a unique value used to access specific data in the control table via Instruction Packets.  
To read or write device data, the address of the corresponding data must be specified in the Instruction Packet.  
For detailed information about Packets, please refer to DYNAMIXEL Protocol 2.0.  
::: info
Negative numbers follow the Two's complement rule. For detailed explanation of Two's complement, please refer to Wikipedia's Two's complement.  
:::

### Area (EEPROM, RAM)

The control table is divided into EEPROM and RAM areas. The characteristics of each area are as follows:

| Area | Description |
|------|-------------|
| EEPROM | Values in the EEPROM area are preserved even when power is turned off (Non-Volatile). |
| RAM | The RAM area is reset to default values each time power is applied (Volatile). |

### Size

Data size is determined by purpose, ranging from 1 to 4 bytes. When modifying data through Instruction Packets, please verify the size of the corresponding data.  
Consecutive data of 2 bytes or more is recorded according to the Little Endian rule.

### Access Permission

Control table data has two access attributes. `RW` allows both read and write access. `R` has a read-only attribute.  
Data with read-only attributes cannot be changed via WRITE Instructions.  
Read-only attributes (`R`) are primarily used for measurement or monitoring purposes, while read-write attributes (`RW`) are used for device control.

### Initial Value

The default values for the EEPROM area listed in the manual are the initial settings (factory default settings) of the product.  
The default value for the RAM area is the value set when power is applied to the device.

::: info
OMY BASE has an additional EEPROM write condition in operation: EEPROM values can only be changed when `DXL Power Enable (512)` is `0`.  
:::

## Control Table Structure

:::tabs key:omy-unit-control-table
== OMY BASE Unit

| **Address** | **Size (Byte)** | **Area** | **Data Name** | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | --------------- | -------- | ------------- | ---------- | ----------------- | --------- | -------- |
| 0 | 2 | EEPROM | Model Number | R | 220 | - | - |
| 2 | 4 | EEPROM | Model Information | R | - | - | - |
| 6 | 1 | EEPROM | Firmware Version | R | - | - | - |
| 7 | 1 | EEPROM | ID | R | 200 | - | - |
| 8 | 1 | EEPROM | Baud Rate (UART) | RW | 9 | 0 ~ 9 | - |
| 9 | 1 | EEPROM | Baud Rate (Bus) | RW | 9 | 0 ~ 9 | - |
| 10 | 2 | EEPROM | Regenerative Voltage Limit | RW | 2900 | 500 ~ 3000 | 0.01 [V] |
| 20 | 1 | EEPROM | Table Sync ID 1 | RW | 255 | 0 ~ 255 | - |
| 21 | 1 | EEPROM | Table Sync ID 2 | RW | 255 | 0 ~ 255 | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 28 | 1 | EEPROM | Table Sync ID 9 | RW | 255 | 0 ~ 255 | - |
| 29 | 1 | EEPROM | Table Sync ID 10 | RW | 255 | 0 ~ 255 | - |
| 30 | 2 | EEPROM | Table Sync Read Address 1 | RW | 0 | 0 ~ 1,023 | - |
| 32 | 2 | EEPROM | Table Sync Read Address 2 | RW | 0 | 0 ~ 1,023 | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 46 | 2 | EEPROM | Table Sync Read Address 9 | RW | 0 | 0 ~ 1,023 | - |
| 48 | 2 | EEPROM | Table Sync Read Address 10 | RW | 0 | 0 ~ 1,023 | - |
| 50 | 2 | EEPROM | Table Sync Read Size 1 | RW | 0 | 0 ~ 1,024 | - |
| 52 | 2 | EEPROM | Table Sync Read Size 2 | RW | 0 | 0 ~ 1,024 | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 66 | 2 | EEPROM | Table Sync Read Size 9 | RW | 0 | 0 ~ 1,024 | - |
| 68 | 2 | EEPROM | Table Sync Read Size 10 | RW | 0 | 0 ~ 1,024 | - |
| 70 | 2 | EEPROM | Table Sync Write Address 1 | RW | 0 | 0 ~ 1,023 | - |
| 72 | 2 | EEPROM | Table Sync Write Address 2 | RW | 0 | 0 ~ 1,023 | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 86 | 2 | EEPROM | Table Sync Write Address 9 | RW | 0 | 0 ~ 1,023 | - |
| 88 | 2 | EEPROM | Table Sync Write Address 10 | RW | 0 | 0 ~ 1,023 | - |
| 90 | 2 | EEPROM | Table Sync Write Size 1 | RW | 0 | 0 ~ 1,024 | - |
| 92 | 2 | EEPROM | Table Sync Write Size 2 | RW | 0 | 0 ~ 1,024 | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 106 | 2 | EEPROM | Table Sync Write Size 9 | RW | 0 | 0 ~ 1,024 | - |
| 108 | 2 | EEPROM | Table Sync Write Size 10 | RW | 0 | 0 ~ 1,024 | - |
| 112 | 2 | EEPROM | Indirect Address 1 | RW | 816 | 0 ~ 1,023 | - |
| 114 | 2 | EEPROM | Indirect Address 2 | RW | 817 | 0 ~ 1,023 | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 500 | 2 | EEPROM | Indirect Address 195 | RW | 1010 | 0 ~ 1,023 | - |
| 502 | 2 | EEPROM | Indirect Address 196 | RW | 1011 | 0 ~ 1,023 | - |
| 512 | 1 | RAM | DXL Power Enable | RW | 0 | 0 ~ 1 | - |
| 513 | 1 | RAM | Regenerative Voltage Enable | RW | 0 | 0 ~ 1 | - |
| 514 | 1 | RAM | Table Sync Enable | RW | 0 | 0 ~ 1 | - |
| 515 | 1 | RAM | LED Red | RW | 0 | 0 ~ 255 | - |
| 516 | 1 | RAM | LED Green | RW | 0 | 0 ~ 255 | - |
| 517 | 1 | RAM | LED Blue | RW | 0 | 0 ~ 255 | - |
| 518 | 1 | RAM | Status | RW | 0 | 0 ~ 255 | - |
| 519 | 1 | RAM | EMG Power Off | RW | 0 | 0 ~ 1 | - |
| 520 | 2 | RAM | Realtime Tick | R | 0 | 0 ~ 32,767 | 1 [ms] |
| 522 | 2 | RAM | Present Input Voltage | R | - | - | 0.01 [V] |
| 556 | 1 | RAM | Status Return Level | RW | 2 | 0 ~ 2 | - |
| 560 | 1 | RAM | Table Sync Read Data 1 | R | 0 | - | - |
| 561 | 1 | RAM | Table Sync Read Data 2 | R | 0 | - | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 686 | 1 | RAM | Table Sync Read Data 127 | R | 0 | - | - |
| 687 | 1 | RAM | Table Sync Read Data 128 | R | 0 | - | - |
| 688 | 1 | RAM | Table Sync Write Data 1 | RW | 0 | 0 ~ 255 | - |
| 689 | 1 | RAM | Table Sync Write Data 2 | RW | 0 | 0 ~ 255 | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 814 | 1 | RAM | Table Sync Write Data 127 | RW | 0 | 0 ~ 255 | - |
| 815 | 1 | RAM | Table Sync Write Data 128 | RW | 0 | 0 ~ 255 | - |
| 816 | 1 | RAM | Indirect Data 1 | RW | 0 | 0 ~ 255 | - |
| 817 | 1 | RAM | Indirect Data 2 | RW | 0 | 0 ~ 255 | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 1010 | 1 | RAM | Indirect Data 195 | RW | 0 | 0 ~ 255 | - |
| 1011 | 1 | RAM | Indirect Data 196 | RW | 0 | 0 ~ 255 | - |

== OMY END Unit

| **Address** | **Size (Byte)** | **Area** | **Data Name** | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | --------------- | -------- | ------------- | ---------- | ----------------- | --------- | -------- |
| 0 | 2 | EEPROM | Model Number | R | 230 | - | - |
| 2 | 4 | EEPROM | Model Information | R | - | - | - |
| 6 | 1 | EEPROM | Firmware Version | R | - | - | - |
| 7 | 1 | EEPROM | ID | R | 210 | - | - |
| 8 | 1 | EEPROM | Baud Rate (Bus) | RW | 6 | 0 ~ 8 | - |
| 10 | 1 | EEPROM | Tool Analog Input Enable | RW | 0 | 0 ~ 1 | - |
| 11 | 1 | EEPROM | Tool Baud Rate | RW | 3 | 0 ~ 10 | - |
| 12 | 1 | EEPROM | Tool Protocol Type | RW | 0 | 0 ~ 1 | - |
| 13 | 1 | EEPROM | Tool Protocol ID | RW | 1 | 0 ~ 252 | - |
| 14 | 1 | EEPROM | Tool Status | R | - | - | - |
| 32 | 1 | RAM | LED Red | RW | 0 | 0 ~ 255 | - |
| 33 | 1 | RAM | LED Green | RW | 0 | 0 ~ 255 | - |
| 34 | 1 | RAM | LED Blue | RW | 0 | 0 ~ 255 | - |
| 36 | 2 | RAM | Realtime Tick | R | 0 | 0 ~ 32,767 | 1 [ms] |
| 40 | 1 | RAM | Digital Input | R | - | - | - |
| 41 | 1 | RAM | Digital Output | RW | 0 | 0 ~ 3 | - |
| 42 | 2 | RAM | Analog Input 0 | R | - | - | 0.01 [V] |
| 44 | 2 | RAM | Analog Input 1 | R | - | - | 0.01 [V] |
| 46 | 1 | RAM | Table Sync Enable | RW | 0 | 0 ~ 1 | - |
| 48 | 2 | EEPROM | Table Sync Read Address 1 | RW | 0 | 0 ~ 1,023 | - |
| 50 | 2 | EEPROM | Table Sync Read Address 2 | RW | 0 | 0 ~ 1,023 | - |
| 52 | 2 | EEPROM | Table Sync Read Size 1 | RW | 0 | 0 ~ 1,024 | - |
| 54 | 2 | EEPROM | Table Sync Read Size 2 | RW | 0 | 0 ~ 1,024 | - |
| 56 | 2 | EEPROM | Table Sync Write Address 1 | RW | 0 | 0 ~ 1,023 | - |
| 58 | 2 | EEPROM | Table Sync Write Address 2 | RW | 0 | 0 ~ 1,023 | - |
| 60 | 2 | EEPROM | Table Sync Write Size 1 | RW | 0 | 0 ~ 1,024 | - |
| 62 | 2 | EEPROM | Table Sync Write Size 2 | RW | 0 | 0 ~ 1,024 | - |
| 72 | 1 | RAM | Table Sync Read Data 1 | R | 0 | - | - |
| 73 | 1 | RAM | Table Sync Read Data 2 | R | 0 | - | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 198 | 1 | RAM | Table Sync Read Data 127 | R | 0 | - | - |
| 199 | 1 | RAM | Table Sync Read Data 128 | R | 0 | - | - |
| 200 | 1 | RAM | Table Sync Write Data 1 | RW | 0 | 0 ~ 255 | - |
| 201 | 1 | RAM | Table Sync Write Data 2 | RW | 0 | 0 ~ 255 | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 326 | 1 | RAM | Table Sync Write Data 127 | RW | 0 | 0 ~ 255 | - |
| 327 | 1 | RAM | Table Sync Write Data 128 | RW | 0 | 0 ~ 255 | - |
| 378 | 2 | EEPROM | Indirect Address 1 | RW | 634 | 0 ~ 1,023 | - |
| 380 | 2 | EEPROM | Indirect Address 2 | RW | 635 | 0 ~ 1,023 | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 630 | 2 | EEPROM | Indirect Address 127 | RW | 760 | 0 ~ 1,023 | - |
| 632 | 2 | EEPROM | Indirect Address 128 | RW | 761 | 0 ~ 1,023 | - |
| 634 | 1 | RAM | Indirect Data 1 | RW | 0 | 0 ~ 255 | - |
| 635 | 1 | RAM | Indirect Data 2 | RW | 0 | 0 ~ 255 | - |
| ... | ... | ... | ... | ... | ... | ... | ... |
| 760 | 1 | RAM | Indirect Data 127 | RW | 0 | 0 ~ 255 | - |
| 761 | 1 | RAM | Indirect Data 128 | RW | 0 | 0 ~ 255 | - |
:::

## Control Table Description

::::tabs key:omy-unit-control-description
== OMY BASE Unit

### Model Number (0)
The model number of the device.

### Model Information (2)
Additional information about the device model.

### Firmware Version (6)
The firmware version of the device.

### ID (7)
A unique number used to identify the device via Instruction Packets. For BASE, the ID is fixed to **200 (0xC8)** (read-only) and does not respond to Broadcast ID **254 (0xFE)**.

::: info
If the Instruction Packet ID is not 200 (0xC8), the unit operates in **Bypass** mode and communicates with DYNAMIXELs connected to the DYNAMIXEL port.
:::

### Baud Rate (UART) (8)
Communication speed between host controller (e.g. Raspberry Pi) and BASE unit.

::: warning
When communicating with a PC via the Debug Port (USB-C), communication with the host controller is not available.
:::

### Baud Rate (Bus) (9)
Communication speed for the DYNAMIXEL bus.

::: info
Communication is only reliable when the baud rate error is **3% or less**.
:::

### Regenerative Voltage Limit / Regenerative Voltage Enable (10 / 513)
Limits voltage rise from regeneration when DYNAMIXELs decelerate or back-drive.

::: warning
Set Regenerative Voltage Limit higher than the power supply output voltage. Lower values may damage internal resistor circuitry.
:::

### Table Sync (ID 20-29, Addr/Size 30-108, Enable 514, Read/Write Data 560-815)
BASE maps connected DYNAMIXEL control tables into local read/write sync buffers with per-ID slots (up to 10).

::: info
Example: IDs 1 and 2, Present Position read / Goal Position write - set Read Address **552**, Write Address **532**, Read Size **4**, Write Size **4**, then enable Table Sync.
:::

### Indirect Address / Indirect Data (112-502 / 816-1011)
Maps specific control table addresses into Indirect Data. For multi-byte items, assign all consecutive bytes.

### DXL Power Enable (512)
Supplies power to connected DYNAMIXELs.

### LED Red / Green / Blue (515-517)
RGB LED intensity (0-255 per channel).

### Status (518)
| Bit | Name | Description |
|-----|------|-------------|
| 3 | Overload Error | Regenerative resistor overload |
| 2 | Dynamixel Error | DYNAMIXEL error state |
| 1 | Communication Error | Communication error |
| 0 | Emergency | Emergency (EMG) state |

### EMG Power Off (519)
Controls whether DYNAMIXEL power is cut when EMG is active.

### Realtime Tick (520)
Device time counter for timestamps. Unit: **1 [ms]**, range **0 ~ 32,767**, then wraps to `0`.

### Present Input Voltage (522)
Current supply voltage to BASE device (0.01 [V] per LSB).

### Status Return Level (556)
Determines which Instructions return a Status Packet (`0`: PING only, `1`: PING + READ, `2`: all).

== OMY END Unit

### Model Number (0)
The model number of the device.

### Model Information (2)
Additional information about the device model.

### Firmware Version (6)
The firmware version of the device.

### ID (7)
A unique number used to identify the device via Instruction Packets. For END, the ID is fixed to **210 (0xD2)** (read-only).

### Baud Rate (Bus) (8)
RS485 communication speed between the controller and END unit.

::: info
Communication is only reliable when the baud rate error is **3% or less**.
:::

### Tool Analog Input Enable (10)
Selects M8 connector mode: **0** (default) = RS485 on pins 1-2; **1** = analog inputs.

::: warning
Configure before connecting a device to M8 connector; changing mode while connected may cause damage.
:::

### Tool Baud Rate / Tool Protocol Type / Tool Protocol ID (11 / 12 / 13)
RS485 speed and protocol for M8 tool port.

::: info
Change Tool Protocol Type (12) only while Table Sync Enable (46) is `0`.
:::

### Tool Status (14)
RS485 communication status for M8 port. Writing `0` clears latched status.

### LED Red / Green / Blue (32-34)
RGB LED intensity (0-255 per channel).

### Realtime Tick (36)
Device time counter for timestamps. Unit: **1 [ms]**, range **0 ~ 32,767**, then wraps to `0`.

### Digital Input / Digital Output (40 / 41)
M8 digital I/O: DI1/DI2 input bits and DO1/DO2 output bits.

### Analog Input 0, 1 (42-45)
Analog voltages on M8 connector when Tool Analog Input mode is enabled.

### Table Sync (Enable 46, Addr/Size 48-62, Read/Write Data 72-327)
END maps DYNAMIXEL control table regions into local Table Sync read/write buffers (up to 2 slots).

::: info
Example: RH-P12-RN Present Position / Goal Position - set Read Address **580**, Read Size **4**, Write Address **564**, Write Size **4**, then enable Table Sync.
:::

### Indirect Address / Indirect Data (378-632 / 634-761)
Maps specific control table addresses into Indirect Data. For multi-byte items, assign all consecutive bytes.
::::

</div>
