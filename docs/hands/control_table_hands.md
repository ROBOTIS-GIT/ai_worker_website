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
  width: 100%;
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
  
  function addAreaClasses() {
    const tables = document.querySelectorAll('.control-table-page table')
    tables.forEach(table => {
      const rows = table.querySelectorAll('tr')
      rows.forEach(row => {
        const cells = row.querySelectorAll('td')
        if (cells.length >= 4) {
          const areaCell = cells[3] // 4th column (Area)
          const areaText = areaCell.textContent.trim()

          if (areaText === 'RAM') {
            row.classList.add('area-ram')
            areaCell.classList.add('area-ram')
          } else if (areaText === 'EEPROM') {
            row.classList.add('area-eeprom')
            areaCell.classList.add('area-eeprom')
          }
        }
      })
    })
  }

  // Run on page load
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', addAreaClasses)
  } else {
    addAreaClasses()
  }

  // Also run after navigation (for SPA)
  if (typeof window !== 'undefined' && window.addEventListener) {
    window.addEventListener('load', addAreaClasses)
    // For VitePress navigation
    setTimeout(addAreaClasses, 100)
  }
})()
</script>

<div class="control-table-page">

# Hands Control Table

The control table consists of multiple data fields that represent the current status of the device and are necessary for operation and control.  
Users can read specific data from the control table through Instruction Packets (READ Instruction) to understand the device status, and control the device by modifying data (WRITE Instruction).

## Control Table, Data, and Address

The control table is a collection of multiple data fields for device status and control.  
Users can read specific data from the control table through READ Instruction Packets to understand the device status.  
Users can also control the device by modifying specific data in the control table through WRITE Instruction Packets.  
Address is a unique value used to access specific data in the control table via Instruction Packets.  
To read or write device data, the address of the corresponding data must be specified in the Instruction Packet.  
For detailed information about Packets, please refer to DYNAMIXEL Protocol 2.0.  
::: info
**Note**: Negative numbers follow the Two's complement rule. For detailed explanation of Two's complement, please refer to Wikipedia's Two's complement.  
:::


### Area (EEPROM, RAM)
The control table is divided into EEPROM, RAM, and Hybrid areas. The characteristics of each area are as follows:
| Area | Description |
|----|-----------|
| EEPROM | Values in the EEPROM area are preserved even when power is turned off (Non-Volatile). |
| RAM | The RAM area is reset to default values each time power is applied (Volatile). |


### Size
Data size is determined by purpose, ranging from 1 to 4 bytes. When modifying data through Instruction Packets, please verify the size of the corresponding data.  
Consecutive data of 2 bytes or more is recorded according to the Little Endian rule.

### Access Permission
Control table data has two access attributes. 'RW' allows both read and write access. 'R' has a read-only attribute.  
Data with read-only attributes cannot be changed via WRITE Instructions.  
Read-only attributes ('R') are primarily used for measurement or monitoring purposes, while read-write attributes ('RW') are used for device control.  

### Initial Value
The default values for the EEPROM area listed in the manual are the initial settings (factory default settings) of the product.  
If modified by the user, the default value becomes the value changed by the user.  
The default value for the RAM area is the value set when power is applied to the device.  

## Control Table Structure

:::tabs key:hand-type
== HX5-D20 Type
| **Address** | **Size (Byte)**  | **Modbus Address**  | **Area** | **Data Name**              | **Access** | **Initial Value** | **Range**   | **Unit**  |
| ----------- | ---------------- | ------------------- | -------- | -------------------        | ---------- | ----------------- | ---------   | --------  |
| 0           | 2                | 40001               | EEPROM   | Model Number               | R          | 260               | -           | -         |
| 2           | 4                | 40002               | EEPROM   | Model Information          | R          | -                 | -           | -         |
| 6           | 1                | 40004 (Lo byte)     | EEPROM   | Firmware Version           | R          | -                 | -           | -         |
| 7           | 1                | 40004 (Hi byte)     | EEPROM   | ID                         | RW         | 10                | 0 ~ 252     | -         |
| 8           | 1                | 40005 (Lo byte)     | RAM      | Bus Watchdog               | RW         | 0                 | 0 ~ 127     | 20 [msec] |
| 11          | 1                | 40006 (Hi byte)     | EEPROM   | Protocol Type              | RW         | 2                 | 2 ~ 10      | -         |
| 12          | 1                | 40007 (Lo byte)     | EEPROM   | Baud Rate (Bus)            | RW         | 6                 | 0 ~ 8       | -         |
| 13          | 1                | 40007 (Hi byte)     | EEPROM   | Return Delay Time          | RW         | 0                 | 0 ~ 254     | 2 [μsec]  |
| 20          | 1                | 40011 (Lo byte)     | EEPROM   | Baud Rate (DXL)            | RW         | 6                 | 0 ~ 6       | -         |
| 15          | 1                | 40008 (Hi byte)     | RAM      | Status Return Level        | RW         | 2                 | 0 ~ 2       | -         |
| 16          | 1                | 40009 (Lo byte)     | RAM      | Registered Instruction     | R          | 0                 | 0 ~ 1       | -         |
| 33          | 1                | 40017 (Hi byte)     | EEPROM   | Operating Mode             | RW         | 0                 | 0 ~ 1       | -         |
| 60          | 2                | 40031               | EEPROM   | Max Voltage Limit          | RW         | 3200              | 3200 ~ 5500 | 0.01 [V]  |
| 62          | 2                | 40032               | EEPROM   | Min Voltage Limit          | RW         | 1600              | 1600 ~ 5500 | 0.01 [V]  |
| 65          | 1                | 40033 (Hi byte)     | RAM      | LED                        | RW         | 0                 | 0 ~ 1       | -         |
| 70          | 1                | 40036 (Lo byte)     | RAM      | TableSync Enable           | RW         | 0                 | 0 ~ 1       | -         |
| 71          | 1                | 40036 (Hi byte)     | RAM      | Hardware Error Status      | R          | 0                 | -           | -         |
| 100         | 2                | 40051               | RAM      | Realtime Tick              | R          | 0                 | 0 ~ 32,767  | 1 [msec]  |
| 102         | 2                | 40052               | RAM      | Present Input Voltage      | R          | -                 | -           | 0.01 [V]  |
| 106         | 1                | 40054 (Lo byte)     | RAM      | Status                     | R          | 0                 | -           | -         |
| 122         | 2                | N/A                 | EEPROM   | Indirect Address 1         | RW         | 0                 | 512 ~ 1,023 | -         |
| 124         | 2                | N/A                 | EEPROM   | Indirect Address 2         | RW         | 0                 | 512 ~ 1,023 | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 632         | 2                | N/A                 | EEPROM   | Indirect Address 256       | RW         | 0                 | 512 ~ 1,023 | -         |
| 634         | 1                | N/A                 | RAM      | Indirect Data 1            | RW         | 0                 | 0 ~ 255     | -         |
| 635         | 1                | N/A                 | RAM      | Indirect Data 2            | RW         | 0                 | 0 ~ 255     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 889         | 1                | N/A                 | RAM      | Indirect Data 256          | RW         | 0                 | 0 ~ 255     | -         |
| 1024        | 1                | 40513 (Lo byte)     | EEPROM   | TableSync1 ID 1            | RW         | 255               | 0 ~ 252     | -         |
| 1025        | 1                | 40513 (Hi byte)     | EEPROM   | TableSync1 ID 2            | RW         | 255               | 0 ~ 252     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1029        | 1                | 40515 (Hi byte)     | EEPROM   | TableSync1 ID 6            | RW         | 255               | 0 ~ 252     | -         |
| 1030        | 2                | 40516               | EEPROM   | TableSync1 Read Address 1  | RW         | 0                 | 0 ~ 1023    | -         |
| 1032        | 2                | 40517               | EEPROM   | TableSync1 Read Address 2  | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1040        | 2                | 40521               | EEPROM   | TableSync1 Read Address 6  | RW         | 0                 | 0 ~ 1023    | -         |
| 1042        | 2                | 40522               | EEPROM   | TableSync1 Read Size 1     | RW         | 0                 | 0 ~ 1023    | -         |
| 1044        | 2                | 40523               | EEPROM   | TableSync1 Read Size 2     | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1052        | 2                | 40527               | EEPROM   | TableSync1 Read Size 6     | RW         | 0                 | 0 ~ 1023    | -         |
| 1054        | 2                | 40528               | EEPROM   | TableSync1 Write Address 1 | RW         | 0                 | 0 ~ 1023    | -         |
| 1056        | 2                | 40529               | EEPROM   | TableSync1 Write Address 2 | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1064        | 2                | 40533               | EEPROM   | TableSync1 Write Address 6 | RW         | 0                 | 0 ~ 1023    | -         |
| 1066        | 2                | 40534               | EEPROM   | TableSync1 Write Size 1    | RW         | 0                 | 0 ~ 1023    | -         |
| 1068        | 2                | 40535               | EEPROM   | TableSync1 Write Size 2    | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1076        | 2                | 40539               | EEPROM   | TableSync1 Write Size 6    | RW         | 0                 | 0 ~ 1023    | -         |
| 1078        | 1                | 40540 (Lo byte)     | RAM      | TableSync1 Read Data 1     | R          | 0                 | 0 ~ 255     | -         |
| 1079        | 1                | 40540 (Hi byte)     | RAM      | TableSync1 Read Data 2     | R          | 0                 | 0 ~ 255     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1149        | 1                | 40575 (Hi byte)     | RAM      | TableSync1 Read Data 72    | R          | 0                 | 0 ~ 255     | -         |
| 1150        | 1                | 40576 (Lo byte)     | RAM      | TableSync1 Write Data 1    | RW         | 0                 | 0 ~ 255     | -         |
| 1151        | 1                | 40576 (Hi byte)     | RAM      | TableSync1 Write Data 2    | RW         | 0                 | 0 ~ 255     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1221        | 1                | 40611 (Hi byte)     | RAM      | TableSync1 Write Data 72   | RW         | 0                 | 0 ~ 255     | -         |
| 1222        | 1                | 40612 (Lo byte)     | EEPROM   | TableSync2 ID 1            | RW         | 255               | 0 ~ 252     | -         |
| 1223        | 1                | 40612 (Hi byte)     | EEPROM   | TableSync2 ID 2            | RW         | 255               | 0 ~ 252     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1227        | 1                | 40614 (Hi byte)     | EEPROM   | TableSync2 ID 6            | RW         | 255               | 0 ~ 252     | -         |
| 1228        | 2                | 40615               | EEPROM   | TableSync2 Read Address 1  | RW         | 0                 | 0 ~ 1023    | -         |
| 1230        | 2                | 40616               | EEPROM   | TableSync2 Read Address 2  | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1238        | 2                | 40620               | EEPROM   | TableSync2 Read Address 6  | RW         | 0                 | 0 ~ 1023    | -         |
| 1240        | 2                | 40621               | EEPROM   | TableSync2 Read Size 1     | RW         | 0                 | 0 ~ 1023    | -         |
| 1242        | 2                | 40622               | EEPROM   | TableSync2 Read Size 2     | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1250        | 2                | 40626               | EEPROM   | TableSync2 Read Size 6     | RW         | 0                 | 0 ~ 1023    | -         |
| 1252        | 2                | 40627               | EEPROM   | TableSync2 Write Address 1 | RW         | 0                 | 0 ~ 1023    | -         |
| 1254        | 2                | 40628               | EEPROM   | TableSync2 Write Address 2 | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1262        | 2                | 40632               | EEPROM   | TableSync2 Write Address 6 | RW         | 0                 | 0 ~ 1023    | -         |
| 1264        | 2                | 40633               | EEPROM   | TableSync2 Write Size 1    | RW         | 0                 | 0 ~ 1023    | -         |
| 1266        | 2                | 40634               | EEPROM   | TableSync2 Write Size 2    | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1274        | 2                | 40638               | EEPROM   | TableSync2 Write Size 6    | RW         | 0                 | 0 ~ 1023    | -         |
| 1276        | 1                | 40639 (Lo byte)     | RAM      | TableSync2 Read Data 1     | R          | 0                 | 0 ~ 255     | -         |
| 1277        | 1                | 40639 (Hi byte)     | RAM      | TableSync2 Read Data 2     | R          | 0                 | 0 ~ 255     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1347        | 1                | 40674 (Hi byte)     | RAM      | TableSync2 Read Data 72    | R          | 0                 | 0 ~ 255     | -         |
| 1348        | 1                | 40675 (Lo byte)     | RAM      | TableSync2 Write Data 1    | RW         | 0                 | 0 ~ 255     | -         |
| 1349        | 1                | 40675 (Hi byte)     | RAM      | TableSync2 Write Data 2    | RW         | 0                 | 0 ~ 255     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1419        | 1                | 40710 (Hi byte)     | RAM      | TableSync2 Write Data 72   | RW         | 0                 | 0 ~ 255     | -         |
| 1420        | 1                | 40711 (Lo byte)     | EEPROM   | TableSync3 ID 1            | RW         | 255               | 0 ~ 252     | -         |
| 1421        | 1                | 40711 (Hi byte)     | EEPROM   | TableSync3 ID 2            | RW         | 255               | 0 ~ 252     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1425        | 1                | 40713 (Hi byte)     | EEPROM   | TableSync3 ID 6            | RW         | 255               | 0 ~ 252     | -         |
| 1426        | 2                | 40714               | EEPROM   | TableSync3 Read Address 1  | RW         | 0                 | 0 ~ 1023    | -         |
| 1428        | 2                | 40715               | EEPROM   | TableSync3 Read Address 2  | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1436        | 2                | 40719               | EEPROM   | TableSync3 Read Address 6  | RW         | 0                 | 0 ~ 1023    | -         |
| 1438        | 2                | 40720               | EEPROM   | TableSync3 Read Size 1     | RW         | 0                 | 0 ~ 1023    | -         |
| 1440        | 2                | 40721               | EEPROM   | TableSync3 Read Size 2     | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1448        | 2                | 40725               | EEPROM   | TableSync3 Read Size 6     | RW         | 0                 | 0 ~ 1023    | -         |
| 1450        | 2                | 40726               | EEPROM   | TableSync3 Write Address 1 | RW         | 0                 | 0 ~ 1023    | -         |
| 1452        | 2                | 40727               | EEPROM   | TableSync3 Write Address 2 | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1460        | 2                | 40731               | EEPROM   | TableSync3 Write Address 6 | RW         | 0                 | 0 ~ 1023    | -         |
| 1462        | 2                | 40732               | EEPROM   | TableSync3 Write Size 1    | RW         | 0                 | 0 ~ 1023    | -         |
| 1464        | 2                | 40733               | EEPROM   | TableSync3 Write Size 2    | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1472        | 2                | 40737               | EEPROM   | TableSync3 Write Size 6    | RW         | 0                 | 0 ~ 1023    | -         |
| 1474        | 1                | 40738 (Lo byte)     | RAM      | TableSync3 Read Data 1     | R          | 0                 | 0 ~ 255     | -         |
| 1475        | 1                | 40738 (Hi byte)     | RAM      | TableSync3 Read Data 2     | R          | 0                 | 0 ~ 255     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1545        | 1                | 40773 (Hi byte)     | RAM      | TableSync3 Read Data 72    | R          | 0                 | 0 ~ 255     | -         |
| 1546        | 1                | 40774 (Lo byte)     | RAM      | TableSync3 Write Data 1    | RW         | 0                 | 0 ~ 255     | -         |
| 1547        | 1                | 40774 (Hi byte)     | RAM      | TableSync3 Write Data 2    | RW         | 0                 | 0 ~ 255     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1617        | 1                | 40809 (Hi byte)     | RAM      | TableSync3 Write Data 72   | RW         | 0                 | 0 ~ 255     | -         |
| 1618        | 1                | 40810 (Lo byte)     | EEPROM   | TableSync4 ID 1            | RW         | 255               | 0 ~ 252     | -         |
| 1619        | 1                | 40810 (Hi byte)     | EEPROM   | TableSync4 ID 2            | RW         | 255               | 0 ~ 252     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1623        | 1                | 40812 (Hi byte)     | EEPROM   | TableSync4 ID 6            | RW         | 255               | 0 ~ 252     | -         |
| 1624        | 2                | 40813               | EEPROM   | TableSync4 Read Address 1  | RW         | 0                 | 0 ~ 1023    | -         |
| 1626        | 2                | 40814               | EEPROM   | TableSync4 Read Address 2  | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1634        | 2                | 40818               | EEPROM   | TableSync4 Read Address 6  | RW         | 0                 | 0 ~ 1023    | -         |
| 1636        | 2                | 40819               | EEPROM   | TableSync4 Read Size 1     | RW         | 0                 | 0 ~ 1023    | -         |
| 1638        | 2                | 40820               | EEPROM   | TableSync4 Read Size 2     | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1646        | 2                | 40824               | EEPROM   | TableSync4 Read Size 6     | RW         | 0                 | 0 ~ 1023    | -         |
| 1648        | 2                | 40825               | EEPROM   | TableSync4 Write Address 1 | RW         | 0                 | 0 ~ 1023    | -         |
| 1650        | 2                | 40826               | EEPROM   | TableSync4 Write Address 2 | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1658        | 2                | 40830               | EEPROM   | TableSync4 Write Address 6 | RW         | 0                 | 0 ~ 1023    | -         |
| 1660        | 2                | 40831               | EEPROM   | TableSync4 Write Size 1    | RW         | 0                 | 0 ~ 1023    | -         |
| 1662        | 2                | 40832               | EEPROM   | TableSync4 Write Size 2    | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1670        | 2                | 40836               | EEPROM   | TableSync4 Write Size 6    | RW         | 0                 | 0 ~ 1023    | -         |
| 1672        | 1                | 40837 (Lo byte)     | RAM      | TableSync4 Read Data 1     | R          | 0                 | 0 ~ 255     | -         |
| 1673        | 1                | 40837 (Hi byte)     | RAM      | TableSync4 Read Data 2     | R          | 0                 | 0 ~ 255     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1743        | 1                | 40872 (Hi byte)     | RAM      | TableSync4 Read Data 72    | R          | 0                 | 0 ~ 255     | -         |
| 1744        | 1                | 40873 (Lo byte)     | RAM      | TableSync4 Write Data 1    | RW         | 0                 | 0 ~ 255     | -         |
| 1745        | 1                | 40873 (Hi byte)     | RAM      | TableSync4 Write Data 2    | RW         | 0                 | 0 ~ 255     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1815        | 1                | 40908 (Hi byte)     | RAM      | TableSync4 Write Data 72   | RW         | 0                 | 0 ~ 255     | -         |
| 1816        | 1                | 40909 (Lo byte)     | EEPROM   | TableSync5 ID 1            | RW         | 255               | 0 ~ 252     | -         |
| 1817        | 1                | 40909 (Hi byte)     | EEPROM   | TableSync5 ID 2            | RW         | 255               | 0 ~ 252     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1821        | 1                | 40911 (Hi byte)     | EEPROM   | TableSync5 ID 6            | RW         | 255               | 0 ~ 252     | -         |
| 1822        | 2                | 40912               | EEPROM   | TableSync5 Read Address 1  | RW         | 0                 | 0 ~ 1023    | -         |
| 1824        | 2                | 40913               | EEPROM   | TableSync5 Read Address 2  | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1832        | 2                | 40917               | EEPROM   | TableSync5 Read Address 6  | RW         | 0                 | 0 ~ 1023    | -         |
| 1834        | 2                | 40918               | EEPROM   | TableSync5 Read Size 1     | RW         | 0                 | 0 ~ 1023    | -         |
| 1836        | 2                | 40919               | EEPROM   | TableSync5 Read Size 2     | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1844        | 2                | 40923               | EEPROM   | TableSync5 Read Size 6     | RW         | 0                 | 0 ~ 1023    | -         |
| 1846        | 2                | 40924               | EEPROM   | TableSync5 Write Address 1 | RW         | 0                 | 0 ~ 1023    | -         |
| 1848        | 2                | 40925               | EEPROM   | TableSync5 Write Address 2 | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1856        | 2                | 40929               | EEPROM   | TableSync5 Write Address 6 | RW         | 0                 | 0 ~ 1023    | -         |
| 1858        | 2                | 40930               | EEPROM   | TableSync5 Write Size 1    | RW         | 0                 | 0 ~ 1023    | -         |
| 1860        | 2                | 40931               | EEPROM   | TableSync5 Write Size 2    | RW         | 0                 | 0 ~ 1023    | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1868        | 2                | 40935               | EEPROM   | TableSync5 Write Size 6    | RW         | 0                 | 0 ~ 1023    | -         |
| 1870        | 1                | 40936 (Lo byte)     | RAM      | TableSync5 Read Data 1     | R          | 0                 | 0 ~ 255     | -         |
| 1871        | 1                | 40936 (Hi byte)     | RAM      | TableSync5 Read Data 2     | R          | 0                 | 0 ~ 255     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 1941        | 1                | 40971 (Hi byte)     | RAM      | TableSync5 Read Data 72    | R          | 0                 | 0 ~ 255     | -         |
| 1942        | 1                | 40972 (Lo byte)     | RAM      | TableSync5 Write Data 1    | RW         | 0                 | 0 ~ 255     | -         |
| 1943        | 1                | 40972 (Hi byte)     | RAM      | TableSync5 Write Data 2    | RW         | 0                 | 0 ~ 255     | -         |
| ...         | ...              | ...                 | ...      | ...                        | ...        | ...               | ...         | ...       |
| 2013        | 1                | 41007 (Hi byte)     | RAM      | TableSync5 Write Data 72   | RW         | 0                 | 0 ~ 255     | -         |
| 2016        | 1                | 41009 (Lo byte)     | RAM      | Preset Index               | RW         | 0                 | 0 ~ 2       | -         |
| 2017        | 1                | 41009 (Hi byte)     | RAM      | Preset Motion Index        | RW         | 0                 | 0 ~ 100     | 1 [%]     |
| 2018        | 2                | 41010               | RAM      | Preset Motion Time         | RW         | 0                 | 0 ~ 32,737  | 1 [msec]  |

:::

## Control Table Description

### Model Number (0)
The model number of the device.

### Model Information (2)
Additional information about the device model.

### Firmware Version (6)
The firmware version of the device.

### ID (7)
A unique number used to identify the device via Instruction Packets. Values from 0 to 253 (0xFD) can be used, and 254 (0xFE) is specially used as the Broadcast ID. When an Instruction Packet is sent with the Broadcast ID (254, 0xFE), commands can be sent to all devices.  
::: info
**Note**: Be careful not to duplicate IDs of connected devices. If device IDs are duplicated, communication errors will occur and searching for DYNAMIXELs with unique IDs will fail.
:::

::: info
**Note**: When the ID of an Instruction packet is the Broadcast ID (0xFE), Status Packets for Read Instructions or Write Instructions are not returned regardless of the Status Return Level (15) setting. For more details, please refer to the Status Packet section in DYNAMIXEL Protocol 2.0.
:::


### Bus Watchdog (8)
Bus Watchdog (8) is a convenience feature that stops the TableSync function when communication (RS485) between the controller and device is interrupted due to an unspecified error.  
Here, communication refers to all Instruction Packets defined in the protocol.
| | Value      | Description                                           |
|---|---------|-----------------------------------------------|
| Range | 0       | Bus Watchdog function disabled, Bus Watchdog Error cleared    |
| Range | 1 ~ 127 | Bus Watchdog enabled (unit: 20 [msec])               |
| Range | -1      | Bus Watchdog Error status                           |

The Bus Watchdog function monitors the communication interval (time) between the controller and device when TableSync Enable (70) is '1' (TableSync ON).  
If the measured communication interval (time) is greater than the Bus Watchdog (8) setting, the TableSync function stops.  
At this time, Bus Watchdog (8) changes to '-1' (Bus Watchdog Error).  
Changing the Bus Watchdog (8) value to '0' clears the Bus Watchdog Error.  

### Protocol Type (11)
To communicate with DYNAMIXEL, an appropriate protocol type must be selected.  
Please refer to the following table and select the appropriate DYNAMIXEL protocol according to the protocol type of the device to be used together.
| Value         | Type                                   | Description                        |
|------------|--------------------------------------|-----------------------------|
| 2 (default) | DYNAMIXEL Protocol 2.0                | Refer to protocol compatibility table        |
| 10         | Modbus-RTU, Industrial Standard Protocol |                            |




### Baud Rate (Bus) (12)
Communication speed for communication with the upper controller.
| Value        | Communication Speed         | Error Rate      |
|-----------|-------------------|-------------|
| 8         | 6M [bps]          | 0.000 [%]   |
| 7         | 4.5M [bps]        | 0.000 [%]   |
| 6 (default) | 4M [bps]          | 0.000 [%]   |
| 5         | 3M [bps]          | 0.000 [%]   |
| 4         | 2M [bps]          | 0.000 [%]   |
| 3         | 1M [bps]          | 0.000 [%]   |
| 2         | 115,200 [bps]     | 0.000 [%]   |
| 1         | 57,600 [bps]      | 0.000 [%]   |
| 0         | 9,600 [bps]       | 0.000 [%]   |


::: info
**Note**: UART communication is not affected if the baud rate error is within 3 [%].
:::

::: info
**Note**: When using U2D2, lower the USB port response delay time (Latency) for stable communication at high baud rates.
:::


### Return Delay Time (13) 
When DYNAMIXEL receives an Instruction Packet, it waits for Return Delay Time (13) and then returns a Status Packet.  
Values from 0 to 254 (0xFE) can be used, and the unit is 2 [μsec].  
For example, if the value is 10, the Status Packet is returned after 20 [μsec].
| Unit      | Range     | Description                |
|-----------|----------|---------------------|
| 2 [μsec]  | 0 ~ 254  | Default: '0' (0 [μsec]) |


### Status Return Level (15)
Determines the return method of Status Packets.
| Value | Responding Instructions                          | Description                                           |
|----|------------------------------------|---------------------------------------------------|
| 0  | PING Instruction                   | Returns Status Packet only for PING commands            |
| 1  | PING Instruction<br>READ Instruction | Returns Status Packet only for PING and READ commands      |
| 2  | All Instructions                   | Returns Status Packet for all commands               |


::: info
**Note**: When the Instruction Packet ID is the Broadcast ID, Status Packets for Read Instructions or Write Instructions are not returned regardless of the Status Return Level (15) setting. For more details, please refer to the Status Packet section in DYNAMIXEL Protocol 2.0.
:::


### Registered Instruction (16)
Indicates whether Write information has been registered by Reg Write Instruction.
| Value | Description |
|----|-----------|
| 0  | No command registered by REG_WRITE. |
| 1  | Command registered by REG_WRITE exists. |


::: info
**Note**: When an ACTION command is executed, the Registered Instruction (16) value changes to '0'.
:::


### Baud Rate (DXL) (20)
Communication speed for communication with DYNAMIXEL.
| Value | Communication Speed | Error Rate |
|----|-----------|--------|
| 6 (default) | 4M [bps] | 0.000 [%] |
| 5 | 3M [bps] | 0.000 [%] |
| 4 | 2M [bps] | 0.000 [%] |
| 3 | 1M [bps] | 0.000 [%] |
| 2 | 115,200 [bps] | 0.000 [%] |
| 1 | 57,600 [bps] | 0.000 [%] |
| 0 | 9,600 [bps] | 0.000 [%] |


::: info
**Note**: UART communication is not affected if the baud rate error is within 3 [%].
:::



### Operating Mode (33)
Sets the operating mode of the device. Since each operating mode has different characteristics, please set the operating mode appropriate for the system you want to implement.
| Value | Operating Mode | Description |
|----|-----------|-----------|
| 0  | Direct Control Mode | Mode that allows control using TableSync function or direct control of DYNAMIXEL. |
| 1  | Preset Motion Mode | Mode that can perform pre-configured motions using DYNAMIXEL Wizard 2.0. |


::: info
**Note**: In Direct Control Mode, when the TableSync function is disabled, direct access and control of DYNAMIXEL is possible.
:::

### Min/Max Voltage Limit (60, 62)
Upper and lower limits of operating voltage.
When Present Input Voltage (102), which represents the current voltage applied to the device, is outside the range of Max Voltage Limit (60) and Min Voltage Limit (62),
the Input Voltage Error Bit (0x01) of Hardware Error Status (71) is set, and the Status Packet transmits the Alert Bit (0x80) through the Error field.

| Unit | Value | Description |
|------|----|------|
| Approx. 0.01 [V] | 1600 ~ 5500 | 16.0 ~ 55.0 [V] |



### LED (65)
Turns the LED ON/OFF.
| Value | Description |
|----|-----------|
| 0 (default) | Turns LED off. |
| 1 | Turns LED on. |

::: info
**Note**: LED operation according to device status (conditions).
| Status | LED Operation |
|------|-----------|
| Booting | Flashes once |
| Factory Reset | Flashes 4 times |
| Alarm | Flashes |
| Boot Mode | Flashes |
| On | Flashes |
:::

### TableSync Enable (70)
Enables/disables the TableSync function. Be sure to disable TableSync before configuring it.

### Hardware Error Status (71)
The device can detect dangerous situations during operation and protect itself. Since each bit's function is applied with OR logic, multiple settings are possible.  
The controller can check whether the Alert Bit (0x80) is set in the Error field of the Status Packet, or check the current status through Hardware Error Status (71).  
The detectable dangerous situations are shown in the table below.

| Bit | Name | Description |
|---|---|-----------|
| 7 | - | Unused, always '0' |
| 6 | Bus Watchdog Error | When Bus Watchdog Error occurs |
| 5 | - | Unused, always '0' |
| 4 | - | Unused, always '0' |
| 3 | - | Unused, always '0' |
| 2 | - | Unused, always '0' |
| 1 | - | Unused, always '0' |
| 0 | Input Voltage Error | When the applied voltage is outside the set operating voltage range |

### Realtime Tick (100)
A counter representing the device's time.
| Unit      | Range         | Description                         |
|-----------|--------------|------------------------------------|
| 1 [msec]  | 0 ~ 32,767   | Restarts from '0' after 32,767. |


### Present Input Voltage (102)
The current supply voltage. 

### Status (106)
Represents the current status of HX.
| Bit | Name | Description |
|----|---|-----------|
| 7 | - | Unused, always '0' |
| 6 | Bus Watchdog Error | When Bus Watchdog Error occurs |
| 5 | - | Unused, always '0' |
| 4 | - | Unused, always '0' |
| 3 | - | Unused, always '0' |
| 2 | - | Unused, always '0' |
| 1 | - | Unused, always '0' |
| 0 | Input Voltage Error | When the applied voltage is outside the set operating voltage range |



### Indirect Address (122~632), Indirect Data (634~889)
Users can use this function to collect and use necessary control tables.  
When a specific address is set in the Indirect Address Table, the Indirect Data Table will have the same function as that specific address.  
For example, if '65' is written to Indirect Address 1 (122) and '1' is written to Indirect Data 1 (634), the LED will turn on. The value of LED (65) will also be written as '1'.  
Also, if a value is written to LED (65), the value of Indirect Data 1 will change in the same way. When a specific address is set in Indirect Address, Indirect Data becomes the same table as that address.  
A point to note is when setting a Control Table with a length of 2 bytes or more as an Indirect Address.  
All bytes of the Control Table Item must be set as Indirect Addresses for normal operation.  
For example, to use Indirect Data 2 as TableSync1 Read Address 1 (1030), it must be set as follows.

::: info example
**Example 1**: Assigning 1-byte LED (65) to Indirect Data 1 (634).  
Indirect Address 1 (122): Change to '65', which is the address value of LED.  
Change Indirect Data 1 (634) to '1': LED (65) value also changes to '1' and LED turns on.  
Change Indirect Data 1 (634) to '0': LED (65) value also changes to '0' and LED turns off.
:::

::: info example
**Example 2**: To assign 2-byte TableSync1 Read Address 1 (1030) to Indirect Data 2 (635), all consecutive 2 bytes must be assigned.  
Indirect Address 2 (124): Change value to 1030, which is the first address of TableSync1 Read Address 1.  
Indirect Address 3 (126): Change value to 1031, which is the second address of TableSync1 Read Address 1.  
Change 2 bytes from Indirect Data 2 to 3 to 561 (0x0231): TableSync1 Read Address 1 (1030) also changes to 561 (0x0231).

| Indirect Data Address | TableSync1 Read Address 1 Address | Stored HEX Value |
|----|------|------|
| 635 | 1030 | 0x31 |
| 636 | 1031 | 0x02 |
:::

::: info
**Note**: To assign data of 2 bytes or more to Indirect Address, all data addresses must be assigned to Indirect Address as in Example 2.
:::


### TableSync1 ID (1024~1029), TableSync2 ID (1222~1227), TableSync3 ID (1420~1425), TableSync4 ID (1618~1623), TableSync5 ID (1816~1821), TableSync1 Read Address (1030~1040), TableSync2 Read Address (1228~1238), TableSync3 Read Address (1426~1436), TableSync4 Read Address (1624~1634), TableSync5 Read Address (1822~1832), TableSync1 Read Size (1042~1052), TableSync2 Read Size (1240~1250), TableSync3 Read Size (1438~1448), TableSync4 Read Size (1636~1646), TableSync5 Read Size (1834~1844), TableSync1 Write Address (1054~1064), TableSync2 Write Address (1252~1262), TableSync3 Write Address (1450~1460), TableSync4 Write Address (1648~1658), TableSync5 Write Address (1846~1856), TableSync1 Write Size (1066~1076), TableSync2 Write Size (1264~1274), TableSync3 Write Size (1462~1472), TableSync4 Write Size (1660~1670), TableSync5 Write Size (1858~1868), TableSync1 Read Data (1078~1149), TableSync2 Read Data (1276~1347), TableSync3 Read Data (1474~1545), TableSync4 Read Data (1672~1743), TableSync5 Read Data (1870~1941), TableSync1 Write Data (1150~1221), TableSync2 Write Data (1348~1419), TableSync3 Write Data (1546~1617), TableSync4 Write Data (1744~1815), TableSync5 Write Data (1942~2013)  

Users can use this function to map specific areas of DYNAMIXEL's control table to HX's Table Sync Read Data and Table Sync Write Data areas.  
HX's TableSync consists of 5 channels in total, with channel 1 assigned to the thumb, channel 2 to the index finger, channel 3 to the middle finger, channel 4 to the ring finger, and channel 5 to the little finger.  
Specify the DYNAMIXEL ID to be mapped in TableSync ID, and specify the start address and size to read from DYNAMIXEL using TableSync Read Address and TableSync Read Size. Then data will be mapped to TableSync Read Data in the order of ID settings. Also, specify the start address and size of data to write to DYNAMIXEL using TableSync Write Address and TableSync Write Size. Then TableSync Write Data will be mapped in the order of ID settings. After configuration, setting TableSync Enable to 1 activates the TableSync function, and TableSync Read Data is constantly updated, while TableSync Write Data updates the mapped control table of DYNAMIXEL when data is written.  
If the TableSync ID value is 255, that slot is disabled. A maximum of 6 slots can be mapped per channel.  
For example, to map Present Position and Goal Position of 2 DYNAMIXELs with IDs 1 and 2 to TableSync1 Read Data and TableSync1 Write Data, configure as follows.

::: info
 TableSync1 ID 1 : 1  
 TableSync1 ID 2 : 2  
 TableSync1 Read Address 1 : 132  
 TableSync1 Read Address 2 : 116  
 TableSync1 Read Size 1 : 4  
 TableSync1 Read Size 2 : 4  
 TableSync1 Write Address 1 : 132  
 TableSync1 Write Address 2 : 116  
 TableSync1 Write Size 1 : 4  
 TableSync1 Write Size 2 : 4
:::
When TableSync Enable is set to 1, TableSync1 Read Data 1~4 will be the same as ID1's Present Position value, and TableSync Read Data 5~8 will be the same as ID2's Present Position value. Also, writing values to TableSync Write Data 1~4 will write to ID1's Goal Position, and writing values to TableSync Write Data 5~8 will write to ID2's Goal Position.

::: info
**Note**: Set TableSync Read Size and TableSync Write Size according to the size of the control table you want to map. Incorrect size settings may cause data corruption.
:::



### Preset Index (2016)
When Operating Mode (33) is in Preset Motion Mode, you can select which of the 3 stored preset motions to perform.  
Editing and saving preset motions is possible in the HX Hands menu of DYNAMIXEL Wizard 2.0.

### Preset Motion Index (2017)
A function that can perform linearly interpolated motion between the start and end motions of the preset motion selected in Preset Index (2016).  
At 0%, the start motion is performed, and at 100%, the end motion is performed.

### Preset Motion Time (2018)
When playing the preset motion selected in Preset Index (2016), the delay time between each motion playback can be set.

</div>
