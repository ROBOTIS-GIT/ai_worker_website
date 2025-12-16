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

/* EEPROM/ROM region styling - subtle green tint */
.control-table-page table tr.area-eeprom {
  background-color: rgba(34, 197, 94, 0.05);
}

.control-table-page table tr.area-eeprom:hover {
  background-color: rgba(34, 197, 94, 0.12);
}

/* Area column badge styling */
.control-table-page table td.area-ram {
  color: #3b82f6;
  font-weight: 600;
}

.control-table-page table td.area-eeprom {
  color: #22c55e;
  font-weight: 600;
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
  background-color: rgba(34, 197, 94, 0.15);
  border-color: #22c55e;
}
</style>

<script>
(function() {
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

# Hands Control Table (Modbus Register Map)

This page lists the full Modbus control table for the robot hand actuator, organized by **function** for easier reference.

<div class="area-legend">
  <div class="area-legend-item">
    <div class="area-legend-color ram"></div>
    <span><strong>RAM</strong> - Volatile memory (runtime data)</span>
  </div>
  <div class="area-legend-item">
    <div class="area-legend-color eeprom"></div>
    <span><strong>EEPROM</strong> - Non-volatile memory (persistent settings)</span>
  </div>
</div>

> **Note**  
> - Address units are in **bytes** unless otherwise noted.  
> - Modbus addresses follow the **4xxxx (Holding Register)** convention.  
> - **RAM** regions are highlighted in blue, **EEPROM** regions in green.

---

## 1. Basic Registers

### 1.1 Identification

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**       | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | ------------------- | ---------- | ----------------- | --------- | -------- |
| 0           | 2               | 40001              | EEPROM   | Model Number        | R          | 260               | -         | -        |
| 2           | 4               | 40002              | EEPROM   | Model Information   | R          | -                 | -         | -        |
| 6           | 1               | 40004 (Lo byte)    | EEPROM   | Firmware Version    | R          | -                 | -         | -        |
| 7           | 1               | 40004 (Hi byte)    | EEPROM   | ID                  | RW         | 10                | 0 ~ 252   | -        |

### 1.2 Communication

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**       | **Access** | **Initial Value** | **Range** | **Unit**   |
| ----------- | ----------------| -------------------| -------- | ------------------- | ---------- | ----------------- | --------- | ---------- |
| 8           | 1               | 40005 (Lo byte)    | RAM      | Bus Watchdog        | RW         | 0                 | 0 ~ 127   | 20 [msec]  |
| 11          | 1               | 40006 (Hi byte)    | EEPROM   | Protocol Type       | RW         | 2                 | 2 ~ 10    | -          |
| 12          | 1               | 40007 (Lo byte)    | EEPROM   | Baud Rate (Bus)     | RW         | 6                 | 0 ~ 8     | -          |
| 13          | 1               | 40007 (Hi byte)    | EEPROM   | Return Delay Time   | RW         | 0                 | 0 ~ 254   | 2 [μsec]   |
| 20          | 1               | 40011 (Lo byte)    | EEPROM   | Baud Rate (DXL)     | RW         | 6                 | 0 ~ 6     | -          |

### 1.3 Response & Status

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**           | **Access** | **Initial Value** | **Range**      | **Unit**  |
| ----------- | ----------------| -------------------| -------- | ----------------------- | ---------- | ----------------- | -------------- | --------- |
| 15          | 1               | 40008 (Hi byte)    | RAM      | Status Return Level     | RW         | 2                 | 0 ~ 2          | -         |
| 16          | 1               | 40009 (Lo byte)    | RAM      | Registered Instruction  | R          | 0                 | 0 ~ 1          | -         |
| 33          | 1               | 40017 (Hi byte)    | EEPROM   | Operating Mode         | RW         | 0                 | 0 ~ 1          | -         |
| 60          | 2               | 40031              | EEPROM   | Max Voltage Limit       | RW         | 3200              | 3200 ~ 5500    | 0.01 [V]  |
| 62          | 2               | 40032              | EEPROM   | Min Voltage Limit       | RW         | 1600              | 1600 ~ 5500    | 0.01 [V]  |
| 65          | 1               | 40033 (Hi byte)    | RAM      | LED                     | RW         | 0                 | 0 ~ 1          | -         |
| 70          | 1               | 40036 (Lo byte)    | RAM      | TableSync Enable        | RW         | 0                 | 0 ~ 1          | -         |
| 71          | 1               | 40036 (Hi byte)    | RAM      | Hardware Error Status | R          | 0                 | -              | -         |
| 100         | 2               | 40051              | RAM      | Realtime Tick           | R          | 0                 | 0 ~ 32,767     | 1 [msec]  |
| 102         | 2               | 40052              | RAM      | Present Input Voltage   | R          | -                 | -              | 0.01 [V]  |
| 106         | 1               | 40054 (Lo byte)    | RAM      | Status                  | R          | 0                 | -              | -         |

---

## 2. Indirect Address & Data

### 2.1 Indirect Address (EEPROM)

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**           | **Access** | **Initial Value** | **Range**      | **Unit** |
| ----------- | ----------------| -------------------| -------- | ----------------------- | ---------- | ----------------- | -------------- | -------- |
| 122         | 2               | N/A                | EEPROM   | Indirect Address 1      | RW         | 0                 | 512 ~ 1,023    | -        |
| 124         | 2               | N/A                | EEPROM   | Indirect Address 2      | RW         | 0                 | 512 ~ 1,023    | -        |
| ...         | ...             | ...                | ...      | ...                     | ...        | ...               | ...            | ...      |
| 632         | 2               | N/A                | EEPROM   | Indirect Address 256    | RW         | 0                 | 512 ~ 1,023    | -        |

### 2.2 Indirect Data (RAM)

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**        | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | -------------------- | ---------- | ----------------- | --------- | -------- |
| 634         | 1               | N/A                | RAM      | Indirect Data 1      | RW         | 0                 | 0 ~ 255   | -        |
| 635         | 1           | N/A                | RAM      | Indirect Data 2      | RW         | 0                 | 0 ~ 255   | -        |
| ...         | ...             | ...                | ...      | ...                  | ...        | ...               | ...       | ...      |
| 889         | 1               | N/A                | RAM      | Indirect Data 256    | RW         | 0                 | 0 ~ 255   | -        |

---

## 3. TableSync1

### 3.1 ID Registers

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**         | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | --------------------- | ---------- | ----------------- | --------- | -------- |
| 1024        | 1               | 40513 (Lo byte)    | EEPROM   | TableSync1 ID 1       | RW         | 255               | 0 ~ 252   | -        |
| 1025        | 1               | 40513 (Hi byte)    | EEPROM   | TableSync1 ID 2       | RW         | 255               | 0 ~ 252   | -        |
| ...         | ...             | ...                | ...      | ...                   | ...        | ...               | ...       | ...      |
| 1029        | 1               | 40515 (Hi byte)    | EEPROM   | TableSync1 ID 6       | RW         | 255               | 0 ~ 252   | -        |

### 3.2 Read Address / Size

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                    | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | -------------------------------- | ---------- | ----------------- | --------- | -------- |
| 1030        | 2               | 40516              | EEPROM   | TableSync1 Read Address 1        | RW         | 0                 | 0 ~ 1023  | -        |
| 1032        | 2               | 40517              | EEPROM   | TableSync1 Read Address 2        | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1040        | 2               | 40521              | EEPROM   | TableSync1 Read Address 6        | RW         | 0                 | 0 ~ 1023  | -        |
| 1042        | 2               | 40522              | EEPROM   | TableSync1 Read Size 1           | RW         | 0                 | 0 ~ 1023  | -        |
| 1044        | 2               | 40523              | EEPROM   | TableSync1 Read Size 2           | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1052        | 2               | 40527              | EEPROM   | TableSync1 Read Size 6           | RW         | 0                 | 0 ~ 1023  | -        |

### 3.3 Write Address / Size

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                    | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | -------------------------------- | ---------- | ----------------- | --------- | -------- |
| 1054        | 2               | 40528              | EEPROM   | TableSync1 Write Address 1       | RW         | 0                 | 0 ~ 1023  | -        |
| 1056        | 2               | 40529              | EEPROM   | TableSync1 Write Address 2       | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1064        | 2               | 40533              | EEPROM   | TableSync1 Write Address 6       | RW         | 0                 | 0 ~ 1023  | -        |
| 1066        | 2               | 40534              | EEPROM   | TableSync1 Write Size 1          | RW         | 0                 | 0 ~ 1023  | -        |
| 1068        | 2               | 40535              | EEPROM   | TableSync1 Write Size 2          | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1076        | 2               | 40539              | EEPROM   | TableSync1 Write Size 6          | RW         | 0                 | 0 ~ 1023  | -        |

### 3.4 Read / Write Data

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                  | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | ------------------------------ | ---------- | ----------------- | --------- | -------- |
| 1078        | 1               | 40540 (Lo byte)    | RAM      | TableSync1 Read Data 1         | R          | 0                 | 0 ~ 255   | -        |
| 1079        | 1               | 40540 (Hi byte)    | RAM      | TableSync1 Read Data 2         | R          | 0                 | 0 ~ 255   | -        |
| ...         | ...             | ...                | ...      | ...                            | ...        | ...               | ...       | ...      |
| 1149        | 1               | 40575 (Hi byte)    | RAM      | TableSync1 Read Data 72        | R          | 0                 | 0 ~ 255   | -        |
| 1150        | 1               | 40576 (Lo byte)    | RAM      | TableSync1 Write Data 1        | RW         | 0                 | 0 ~ 255   | -        |
| 1151        | 1               | 40576 (Hi byte)    | RAM      | TableSync1 Write Data 2        | RW         | 0                 | 0 ~ 255   | -        |
| ...         | ...             | ...                | ...      | ...                            | ...        | ...               | ...       | ...      |
| 1221        | 1               | 40611 (Hi byte)    | RAM      | TableSync1 Write Data 72       | RW         | 0                 | 0 ~ 255   | -        |

---

## 4. TableSync2

구조는 TableSync1과 동일하며, 베이스 주소만 다릅니다.

### 4.1 ID Registers

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**         | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | --------------------- | ---------- | ----------------- | --------- | -------- |
| 1222        | 1               | 40612 (Lo byte)    | EEPROM   | TableSync2 ID 1       | RW         | 255               | 0 ~ 252   | -        |
| 1223        | 1               | 40612 (Hi byte)    | EEPROM   | TableSync2 ID 2       | RW         | 255               | 0 ~ 252   | -        |
| ...         | ...             | ...                | ...      | ...                   | ...        | ...               | ...       | ...      |
| 1227        | 1               | 40614 (Hi byte)    | EEPROM   | TableSync2 ID 6       | RW         | 255               | 0 ~ 252   | -        |

### 4.2 Read Address / Size

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                    | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | -------------------------------- | ---------- | ----------------- | --------- | -------- |
| 1228        | 2               | 40615              | EEPROM   | TableSync2 Read Address 1        | RW         | 0                 | 0 ~ 1023  | -        |
| 1230        | 2               | 40616              | EEPROM   | TableSync2 Read Address 2        | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1238        | 2               | 40620              | EEPROM   | TableSync2 Read Address 6        | RW         | 0                 | 0 ~ 1023  | -        |
| 1240        | 2               | 40621              | EEPROM   | TableSync2 Read Size 1           | RW         | 0                 | 0 ~ 1023  | -        |
| 1242        | 2               | 40622              | EEPROM   | TableSync2 Read Size 2           | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1250        | 2               | 40626              | EEPROM   | TableSync2 Read Size 6           | RW         | 0                 | 0 ~ 1023  | -        |

### 4.3 Write Address / Size

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                    | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | -------------------------------- | ---------- | ----------------- | --------- | -------- |
| 1252        | 2               | 40627              | EEPROM   | TableSync2 Write Address 1       | RW         | 0                 | 0 ~ 1023  | -        |
| 1254        | 2               | 40628              | EEPROM   | TableSync2 Write Address 2       | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1262        | 2               | 40632              | EEPROM   | TableSync2 Write Address 6       | RW         | 0                 | 0 ~ 1023  | -        |
| 1264        | 2               | 40633              | EEPROM   | TableSync2 Write Size 1          | RW         | 0                 | 0 ~ 1023  | -        |
| 1266        | 2               | 40634              | EEPROM   | TableSync2 Write Size 2          | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1274        | 2               | 40638              | EEPROM   | TableSync2 Write Size 6          | RW         | 0                 | 0 ~ 1023  | -        |

### 4.4 Read / Write Data

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                  | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | ------------------------------ | ---------- | ----------------- | --------- | -------- |
| 1276        | 1               | 40639 (Lo byte)    | RAM      | TableSync2 Read Data 1         | R          | 0                 | 0 ~ 255   | -        |
| 1277        | 1               | 40639 (Hi byte)    | RAM      | TableSync2 Read Data 2         | R          | 0                 | 0 ~ 255   | -        |
| ...         | ...             | ...                | ...      | ...                            | ...        | ...               | ...       | ...      |
| 1347        | 1               | 40674 (Hi byte)    | RAM      | TableSync2 Read Data 72        | R          | 0                 | 0 ~ 255   | -        |
| 1348        | 1               | 40675 (Lo byte)    | RAM      | TableSync2 Write Data 1        | RW         | 0                 | 0 ~ 255   | -        |
| 1349        | 1               | 40675 (Hi byte)    | RAM      | TableSync2 Write Data 2        | RW         | 0                 | 0 ~ 255   | -        |
| ...         | ...             | ...                | ...      | ...                            | ...        | ...               | ...       | ...      |
| 1419        | 1               | 40710 (Hi byte)    | RAM      | TableSync2 Write Data 72       | RW         | 0                 | 0 ~ 255   | -        |

---

## 5. TableSync3

### 5.1 ID Registers

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**         | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | --------------------- | ---------- | ----------------- | --------- | -------- |
| 1420        | 1               | 40711 (Lo byte)    | EEPROM   | TableSync3 ID 1       | RW         | 255               | 0 ~ 252   | -        |
| 1421        | 1               | 40711 (Hi byte)    | EEPROM   | TableSync3 ID 2       | RW         | 255               | 0 ~ 252   | -        |
| ...         | ...             | ...                | ...      | ...                   | ...        | ...               | ...       | ...      |
| 1425        | 1               | 40713 (Hi byte)    | EEPROM   | TableSync3 ID 6       | RW         | 255               | 0 ~ 252   | -        |

### 5.2 Read Address / Size

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                    | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | -------------------------------- | ---------- | ----------------- | --------- | -------- |
| 1426        | 2               | 40714              | EEPROM   | TableSync3 Read Address 1        | RW         | 0                 | 0 ~ 1023  | -        |
| 1428        | 2               | 40715              | EEPROM   | TableSync3 Read Address 2        | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1436        | 2               | 40719              | EEPROM   | TableSync3 Read Address 6        | RW         | 0                 | 0 ~ 1023  | -        |
| 1438        | 2               | 40720              | EEPROM   | TableSync3 Read Size 1           | RW         | 0                 | 0 ~ 1023  | -        |
| 1440        | 2               | 40721              | EEPROM   | TableSync3 Read Size 2           | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1448        | 2               | 40725              | EEPROM   | TableSync3 Read Size 6           | RW         | 0                 | 0 ~ 1023  | -        |

### 5.3 Write Address / Size

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                    | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | -------------------------------- | ---------- | ----------------- | --------- | -------- |
| 1450        | 2               | 40726              | EEPROM   | TableSync3 Write Address 1       | RW         | 0                 | 0 ~ 1023  | -        |
| 1452        | 2               | 40727              | EEPROM   | TableSync3 Write Address 2       | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1460        | 2               | 40731              | EEPROM   | TableSync3 Write Address 6       | RW         | 0                 | 0 ~ 1023  | -        |
| 1462        | 2               | 40732              | EEPROM   | TableSync3 Write Size 1          | RW         | 0                 | 0 ~ 1023  | -        |
| 1464        | 2               | 40733              | EEPROM   | TableSync3 Write Size 2          | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1472        | 2               | 40737              | EEPROM   | TableSync3 Write Size 6          | RW         | 0                 | 0 ~ 1023  | -        |

### 5.4 Read / Write Data

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                  | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | ------------------------------ | ---------- | ----------------- | --------- | -------- |
| 1474        | 1               | 40738 (Lo byte)    | RAM      | TableSync3 Read Data 1         | R          | 0                 | 0 ~ 255   | -        |
| 1475        | 1               | 40738 (Hi byte)    | RAM      | TableSync3 Read Data 2         | R          | 0                 | 0 ~ 255   | -        |
| ...         | ...             | ...                | ...      | ...                            | ...        | ...               | ...       | ...      |
| 1545        | 1               | 40773 (Hi byte)    | RAM      | TableSync3 Read Data 72        | R          | 0                 | 0 ~ 255   | -        |
| 1546        | 1               | 40774 (Lo byte)    | RAM      | TableSync3 Write Data 1        | RW         | 0                 | 0 ~ 255   | -        |
| 1547        | 1               | 40774 (Hi byte)    | RAM      | TableSync3 Write Data 2        | RW         | 0                 | 0 ~ 255   | -        |
| ...         | ...             | ...                | ...      | ...                            | ...        | ...               | ...       | ...      |
| 1617        | 1               | 40809 (Hi byte)    | RAM      | TableSync3 Write Data 72       | RW         | 0                 | 0 ~ 255   | -        |

---

## 6. TableSync4

### 6.1 ID Registers

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**         | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | --------------------- | ---------- | ----------------- | --------- | -------- |
| 1618        | 1               | 40810 (Lo byte)    | EEPROM   | TableSync4 ID 1       | RW         | 255               | 0 ~ 252   | -        |
| 1619        | 1               | 40810 (Hi byte)    | EEPROM   | TableSync4 ID 2       | RW         | 255               | 0 ~ 252   | -        |
| ...         | ...             | ...                | ...      | ...                   | ...        | ...               | ...       | ...      |
| 1623        | 1               | 40812 (Hi byte)    | EEPROM   | TableSync4 ID 6       | RW         | 255               | 0 ~ 252   | -        |

### 6.2 Read Address / Size

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                    | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | -------------------------------- | ---------- | ----------------- | --------- | -------- |
| 1624        | 2               | 40813              | EEPROM   | TableSync4 Read Address 1        | RW         | 0                 | 0 ~ 1023  | -        |
| 1626        | 2               | 40814              | EEPROM   | TableSync4 Read Address 2        | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1634        | 2               | 40818              | EEPROM   | TableSync4 Read Address 6        | RW         | 0                 | 0 ~ 1023  | -        |
| 1636        | 2               | 40819              | EEPROM   | TableSync4 Read Size 1           | RW         | 0                 | 0 ~ 1023  | -        |
| 1638        | 2               | 40820              | EEPROM   | TableSync4 Read Size 2           | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1646        | 2               | 40824              | EEPROM   | TableSync4 Read Size 6           | RW         | 0                 | 0 ~ 1023  | -        |

### 6.3 Write Address / Size

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                    | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | -------------------------------- | ---------- | ----------------- | --------- | -------- |
| 1648        | 2               | 40825              | EEPROM   | TableSync4 Write Address 1       | RW         | 0                 | 0 ~ 1023  | -        |
| 1650        | 2               | 40826              | EEPROM   | TableSync4 Write Address 2       | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1658        | 2               | 40830              | EEPROM   | TableSync4 Write Address 6       | RW         | 0                 | 0 ~ 1023  | -        |
| 1660        | 2               | 40831              | EEPROM   | TableSync4 Write Size 1          | RW         | 0                 | 0 ~ 1023  | -        |
| 1662        | 2               | 40832              | EEPROM   | TableSync4 Write Size 2          | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1670        | 2               | 40836              | EEPROM   | TableSync4 Write Size 6          | RW         | 0                 | 0 ~ 1023  | -        |

### 6.4 Read / Write Data

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                  | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | ------------------------------ | ---------- | ----------------- | --------- | -------- |
| 1672        | 1               | 40837 (Lo byte)    | RAM      | TableSync4 Read Data 1         | R          | 0                 | 0 ~ 255   | -        |
| 1673        | 1               | 40837 (Hi byte)    | RAM      | TableSync4 Read Data 2         | R          | 0                 | 0 ~ 255   | -        |
| ...         | ...             | ...                | ...      | ...                            | ...        | ...               | ...       | ...      |
| 1743        | 1               | 40872 (Hi byte)    | RAM      | TableSync4 Read Data 72        | R          | 0                 | 0 ~ 255   | -        |
| 1744        | 1               | 40873 (Lo byte)    | RAM      | TableSync4 Write Data 1        | RW         | 0                 | 0 ~ 255   | -        |
| 1745        | 1               | 40873 (Hi byte)    | RAM      | TableSync4 Write Data 2        | RW         | 0                 | 0 ~ 255   | -        |
| ...         | ...             | ...                | ...      | ...                            | ...        | ...               | ...       | ...      |
| 1815        | 1               | 40908 (Hi byte)    | RAM      | TableSync4 Write Data 72       | RW         | 0                 | 0 ~ 255   | -        |

---

## 7. TableSync5

### 7.1 ID Registers

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**         | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | --------------------- | ---------- | ----------------- | --------- | -------- |
| 1816        | 1               | 40909 (Lo byte)    | EEPROM   | TableSync5 ID 1       | RW         | 255               | 0 ~ 252   | -        |
| 1817        | 1               | 40909 (Hi byte)    | EEPROM   | TableSync5 ID 2       | RW         | 255               | 0 ~ 252   | -        |
| ...         | ...             | ...                | ...      | ...                   | ...        | ...               | ...       | ...      |
| 1821        | 1               | 40911 (Hi byte)    | EEPROM   | TableSync5 ID 6       | RW         | 255               | 0 ~ 252   | -        |

### 7.2 Read Address / Size

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                    | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | -------------------------------- | ---------- | ----------------- | --------- | -------- |
| 1822        | 2               | 40912              | EEPROM   | TableSync5 Read Address 1        | RW         | 0                 | 0 ~ 1023  | -        |
| 1824        | 2               | 40913              | EEPROM   | TableSync5 Read Address 2        | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1832        | 2               | 40917              | EEPROM   | TableSync5 Read Address 6        | RW         | 0                 | 0 ~ 1023  | -        |
| 1834        | 2               | 40918              | EEPROM   | TableSync5 Read Size 1           | RW         | 0                 | 0 ~ 1023  | -        |
| 1836        | 2               | 40919              | EEPROM   | TableSync5 Read Size 2           | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1844        | 2               | 40923              | EEPROM   | TableSync5 Read Size 6           | RW         | 0                 | 0 ~ 1023  | -        |

### 7.3 Write Address / Size

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                    | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | -------------------------------- | ---------- | ----------------- | --------- | -------- |
| 1846        | 2               | 40924              | EEPROM   | TableSync5 Write Address 1       | RW         | 0                 | 0 ~ 1023  | -        |
| 1848        | 2               | 40925              | EEPROM   | TableSync5 Write Address 2       | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1856        | 2               | 40929              | EEPROM   | TableSync5 Write Address 6       | RW         | 0                 | 0 ~ 1023  | -        |
| 1858        | 2               | 40930              | EEPROM   | TableSync5 Write Size 1          | RW         | 0                 | 0 ~ 1023  | -        |
| 1860        | 2               | 40931              | EEPROM   | TableSync5 Write Size 2          | RW         | 0                 | 0 ~ 1023  | -        |
| ...         | ...             | ...                | ...      | ...                              | ...        | ...               | ...       | ...      |
| 1868        | 2               | 40935              | EEPROM   | TableSync5 Write Size 6          | RW         | 0                 | 0 ~ 1023  | -        |

### 7.4 Read / Write Data

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**                  | **Access** | **Initial Value** | **Range** | **Unit** |
| ----------- | ----------------| -------------------| -------- | ------------------------------ | ---------- | ----------------- | --------- | -------- |
| 1870        | 1               | 40936 (Lo byte)    | RAM      | TableSync5 Read Data 1         | R          | 0                 | 0 ~ 255   | -        |
| 1871        | 1               | 40936 (Hi byte)    | RAM      | TableSync5 Read Data 2         | R          | 0                 | 0 ~ 255   | -        |
| ...         | ...             | ...                | ...      | ...                            | ...        | ...               | ...       | ...      |
| 1941        | 1               | 40971 (Hi byte)    | RAM      | TableSync5 Read Data 72        | R          | 0                 | 0 ~ 255   | -        |
| 1942        | 1               | 40972 (Lo byte)    | RAM      | TableSync5 Write Data 1        | RW         | 0                 | 0 ~ 255   | -        |
| 1943        | 1               | 40972 (Hi byte)    | RAM      | TableSync5 Write Data 2        | RW         | 0                 | 0 ~ 255   | -        |
| ...         | ...             | ...                | ...      | ...                            | ...        | ...               | ...       | ...      |
| 2013        | 1               | 41007 (Hi byte)    | RAM      | TableSync5 Write Data 72       | RW         | 0                 | 0 ~ 255   | -        |

---

## 8. Preset Control

| **Address** | **Size (Byte)** | **Modbus Address** | **Area** | **Data Name**           | **Access** | **Initial Value** | **Range**      | **Unit**    |
| ----------- | ----------------| -------------------| -------- | ----------------------- | ---------- | ----------------- | -------------- | ----------- |
| 2016        | 1               | 41009 (Lo byte)    | RAM      | Preset Index            | RW         | 0                 | 0 ~ 2          | -           |
| 2017        | 1               | 41009 (Hi byte)    | RAM      | Preset Motion Index     | RW         | 0                 | 0 ~ 100        | 1 [%]       |
| 2018        | 2               | 41010              | RAM      | Preset Motion Time      | RW         | 0                 | 0 ~ 32,737     | 1 [msec]    |

</div>
