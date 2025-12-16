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

컨트롤 테이블은 장치의 현재 상태와 구동 및 제어에 필요한 다수의 데이터로 이루어져 있습니다.
사용자는 Instruction Packet을 통해 컨트롤 테이블의 특정 데이터를 읽어서(READ Instruction) 장치의 상태를 파악할 수 있고, 데이터를 변경함으로써(WRITE Instruction) 장치를 제어할 수 있습니다.

## 컨트롤 테이블, 데이터, 주소

컨트롤 테이블은 장치의 상태와 제어를 위한 다수의 데이터 필드로 구성된 집합체입니다.  
사용자는 READ Instruction Packet을 통해 컨트롤 테이블의 특정 데이터를 읽어서 장치의 상태를 파악할 수 있습니다.  
또한 WRITE Instruction Packet을 통해 컨트롤 테이블의 특정 데이터를 변경함으로써 장치를 제어할 수 있습니다.  
Address는 Instruction Packet으로 컨트롤 테이블의 특정 데이터를 접근할 때 사용하는 고유값입니다.  
장치의 데이터를 읽거나 쓰기 위해서는 Instruction Packet에 해당 데이터의 주소를 지정해 주어야 합니다.  
Packet에 대한 자세한 내용은 다이나믹셀 프로토콜 2.0을 참고해주세요.  
::: info
참고 : 음수의 표현 방법은 2의 보수(Two’s complement) 규칙을 따릅니다. 2의 보수에 대한 자세한 설명은 위키피디아의 Two’s complement를 참고하세요.  
:::


### 영역 (EEPROM, RAM)
컨트롤 테이블은 EEPROM, RAM, Hybrid 영역으로 구분됩니다. 각 영역의 특징은 다음과 같습니다.
| 영역 | 상세 설명 |
|----|-----------|
| EEPROM | EEPROM 영역 값을 변경하면 전원이 꺼져도 그 값이 보존 (Non-Volatile). |
| RAM | RAM 영역은 전원이 인가될 때마다 다시 기본값으로 설정됩니다(Volatile). |


### 크기
데이터의 크기는 용도에 따라 1 ~ 4 byte로 정해져 있습니다. Instruction Packet을 통해 데이터를 변경할 때는 해당 데이터의 크기를 확인하시기 바랍니다.  
2 byte 이상의 연속된 데이터는 Little Endian 규칙에 의해 기록됩니다.

### 접근권한
컨트롤 테이블의 데이터는 2가지 접근 속성을 갖습니다. ‘RW’는 읽기와 쓰기 접근이 모두 가능합니다. 반면 ‘R’은 읽기 전용(Read Only) 속성을 갖습니다.  
읽기 전용 속성의 데이터는 WRITE Instruction으로 값이 변경되지 않습니다.  
읽기 전용 속성(‘R’)은 주로 측정 또는 모니터링 용도로 사용되고, 읽기 쓰기 속성(‘RW’)은 장치의 제어 용도로 사용됩니다.  

### 기본값
매뉴얼에 표기된 EEPROM 영역의 기본값은 제품의 초기 설정값(공장 출하 설정값)입니다.  
사용자가 변경한 경우, 기본값은 사용자가 변경한 값으로 적용됩니다.  
RAM 영역의 기본값은 장치에 전원이 인가되었을 때 설정되는 값입니다.  

## 컨트롤 테이블 구성

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

## 컨트롤 테이블 설명

### Model Number (0)
장치의 모델 번호입니다.

### Model Information (2)
장치 모델에 대한 추가 정보입니다.

### Firmware Version (6)
장치의 펌웨어 버전입니다.

### ID (7)
Instruction Packet으로 장치를 식별하기 위한 고유 번호입니다. 0~253 (0xFD)까지 사용 가능하며, 254(0xFE)는 브로드캐스트(Broadcast) ID로 특수하게 사용됩니다. 브로드캐스트 ID(254, 0xFE)로 Instruction Packet을 전송하면 모든 장치에 명령을 내릴 수 있습니다.  
::: info
참고 : 연결된 장치의 ID가 중복되지 않도록 주의해야 합니다. 장치의 ID가 중복되면, 통신 오류 및 고유의 ID를 가지는 다이나믹셀 검색에 실패합니다.
:::

::: info
참고 : Instruction packet의 ID가 Broadcast ID(0xFE)인 경우, Status Return Level (15)의 설정값과 무관하게 Read Instruction 또는 Write Instruction에 대한 Status Packet은 반환되지 않습니다. 더 자세한 설명은 다이나믹셀 프로토콜 2.0의 Status Packet 항목을 참조하세요.
:::


### Bus Watchdog (8)
Bus Watchdog(98)은 특정할 수 없는 오류에 의해 제어기와 장치의 통신(RS485)이 단절된 경우, TableSync 기능을 정지시키기 위한 편의기능입니다.  
여기서 통신이란 프로토콜에서 정의된 모든 Instruction Packet을 의미합니다.
| | 값      | 설명                                           |
|---|---------|-----------------------------------------------|
| 범위 | 0       | Bus Watchdog 기능 비활성화, Bus Watchdog Error 해제    |
| 범위 | 1 ~ 127 | Bus Watchdog 활성화 (단위: 20 [msec])               |
| 범위 | -1      | Bus Watchdog Error 상태                           |

Bus Watchdog 기능은 TableSync Enable(70)가 ‘1’(TableSync ON) 인 경우, 제어기와 장치의 통신 간격(시간)을 감시합니다.  
측정된 통신 간격(시간)이 Bus Watchdog(98)의 설정값 보다 클 경우, TableSync 기능은 정지합니다.  
이때 Bus Watchdog(98)은 ‘-1’(Bus Watchdog Error)로 변경됩니다.  
Bus Watchdog(98)의 값을 ‘0’으로 변경하면, Bus Watchdog Error는 해제됩니다.  

### Protocol Type (11)
다이나믹셀과 통신하기 위해서는, 적절한 프로토콜 타입을 선택해야 합니다.  
다음 표를 참고하여, 함께 사용할 장치의 프로토콜 타입에 맞추어, 다이나믹셀의 적절한 프로토콜을 선택하세요.
| 값         | 타입                                   | 설명                        |
|------------|--------------------------------------|-----------------------------|
| 2 (기본값) | DYNAMIXEL Protocol 2.0                | 프로토콜 호환표 참고        |
| 10         | Modbus-RTU, Industrial Standard Protocol |                            |




### Baud Rate(Bus) (12)
상위 제어기와 통신하기 위한 통신 속도 입니다.
| 값        | 통신 속도         | 오차율      |
|-----------|-------------------|-------------|
| 8         | 6M [bps]          | 0.000 [%]   |
| 7         | 4.5M [bps]        | 0.000 [%]   |
| 6(기본값) | 4M [bps]          | 0.000 [%]   |
| 5         | 3M [bps]          | 0.000 [%]   |
| 4         | 2M [bps]          | 0.000 [%]   |
| 3         | 1M [bps]          | 0.000 [%]   |
| 2         | 115,200 [bps]     | 0.000 [%]   |
| 1         | 57,600 [bps]      | 0.000 [%]   |
| 0         | 9,600 [bps]       | 0.000 [%]   |


::: info
참고 : UART는 Baudrate 오차가 3 [%] 이내이면 통신에 지장이 없습니다.
:::

::: info
참고: U2D2을 이용 시, 높은 통신 Baud rate에서 안정적인 통신을 위해서는 USB 포트의 응답지연시간(Latency) 을 낮춰주세요.
:::


### Return Delay Time(13) 
다이나믹셀은 Instruction Packet을 수신하면, Return Delay Time(9) 만큼 대기한 후 Status Packet을 반환 합니다.  
0 ~ 254 (0xFE) 까지 사용 가능하며 단위는 2 [μsec] 입니다.  
예를 들어, 값이 10일 경우 20 [μsec] 만큼 시간이 지난 후에 Status Packet을 반환합니다.
| 단위      | 범위     | 설명                |
|-----------|----------|---------------------|
| 2 [μsec]  | 0 ~ 254  | 기본값: ‘0’(0 [μsec]) |


### Status Return Level (15)
Status Packet의 반환 방식을 결정합니다.
| 값 | 응답하는 명령                          | 상세설명                                           |
|----|------------------------------------|---------------------------------------------------|
| 0  | PING Instruction                   | PING 명령에 대해서만 Status Packet을 반환함            |
| 1  | PING Instruction<br>READ Instruction | PING과 READ 명령에 대해서만 Status Packet을 반환함      |
| 2  | All Instructions                   | 모든 명령에 대해서 Status Packet을 반환함               |


::: info
참고 : Instruction Packet ID가 Broadcast ID 인 경우는 Status Return Level (68)의 설정값과 무관하게 Read Instruction 또는 Write Instruction에 대한 Status Packet은 반환되지 않습니다. 더 자세한 설명은 DYNAMIXEL Protocol 2.0의 Status Packet 항목을 참조하세요.
:::


### Registered Instruction (16)
Reg Write Instruction에 의해서 등록된 Write 정보의 유무를 나타냅니다.
| 값 | 상세 설명 |
|----|-----------|
| 0  | REG_WRITE에 의해 등록된 명령이 없습니다. |
| 1  | REG_WRITE에 의해 등록된 명령이 있습니다. |


::: info
참고 : ACTION 명령을 수행하면 Registered Instruction (69) 값이 ‘0’으로 바뀝니다.
:::


### Baud Rate(DXL) (20)
다이나믹셀과 통신을 하기 위한 통신 속도입니다.
| 값 | 통싱 속도 | 오차율 |
|----|-----------|--------|
| 6(기본값) | 4M [bps] | 0.000 [%] |
| 5 | 3M [bps] | 0.000 [%] |
| 4 | 2M [bps] | 0.000 [%] |
| 3 | 1M [bps] | 0.000 [%] |
| 2 | 115,200 [bps] | 0.000 [%] |
| 1 | 57,600 [bps] | 0.000 [%] |
| 0 | 9,600 [bps] | 0.000 [%] |


::: info
참고 : UART는 Baudrate 오차가 3 [%] 이내이면 통신에 지장이 없습니다.
:::



### Operating Mode (33)
장치의 동작 모드를 설정합니다. 각 동작 모드마다 특성이 다르기 때문에, 구현하려는 시스템에 적합한 동작 모드를 설정하시기 바랍니다.
| 값 | 동작 모드 | 세부 설명 |
|----|-----------|-----------|
| 0  | 직접 제어 모드 | TableSync 기능을 이용한 제어 또는 DYNAMIXEL의 직접 제어가 가능한 모드입니다. |
| 1  | 프리셋 모션 모드 | DYNAMIXEL Wizard 2.0을 이용하여 미리 설정한 동작을 수행할 수 있는 모드입니다. |


::: info
참고 : 직접 제어 모드에서 TableSync 기능(2.3.19 항목) 비활성화 시 DYNAMIXEL의 직접 접근 및 제어가 가능합니다.
:::

### Min/Max Voltage Limit(60, 62)
동작 전압의 상한 값과 하한 값입니다.
장치에 현재 인가된 전압을 나타내는 Present Input Voltage(102)가 Max Voltage Limit(60)와 Min Voltage Limit(62)의 범위를 벗어날 경우,
Hardware Error Status(71)의 Input Voltage Error Bit(0x01)이 설정되고, Status Packet은 Error 필드를 통해서 Alert Bit(0x80)을 전송합니다.

| 단위 | 값 | 설명 |
|------|----|------|
| 약 0.01 [V] | 1600 ~ 5500 | 16.0 ~ 55.0 [V] |



### LED (65)
LED를 ON/OFF 합니다.
| 값 | 설명 |
|----|-----------|
| 0(기본값) | LED를 Off 시킵니다. |
| 1 | LED를 On 시킵니다. |

::: info
**참고** : 장치의 상태(조건)에 따른 LED의 동작입니다.
| 상태 | LED 동작 |
|------|-----------|
| 부팅 | 1회 점멸 |
| 공장 초기화 | 4회 점멸 |
| 알람 | 점멸 |
| 부트 모드 | 점멸 |
| 점등 | 점멸 |
:::

### TableSync Enable (70)
TableSync 기능(2.3.19 항목)을 활성화/비활성화 합니다. TableSync를 설정하기 전에 반드시 비활성화 해주세요.

### Hardware Error Status(71)
장치는 동작 중에 발생하는 위험 상황을 감지하여 스스로를 보호할 수 있습니다. 각 Bit의 기능은 ‘OR’의 논리로 적용되기 때문에 중복 설정이 가능합니다.  
제어기는 Status Packet의 Error 필드에 Alert Bit(0x80)이 설정되었는지를 확인하거나, Hardware Error Status(70) 을 통해서 현재 상태를 확인할 수 있습니다.  
감지할 수 있는 위험 상황은 아래 표와 같습니다.

| Bit | 명칭 | 상세 설명 |
|---|---|-----------|
| 7 | - | 미사용, 항상 ‘0’ |
| 6 | Bus Watchdog Error | Bus Watchdog Error가 발생한 경우 |
| 5 | - | 미사용, 항상 ‘0’ |
| 4 | - | 미사용, 항상 ‘0’ |
| 3 | - | 미사용, 항상 ‘0’ |
| 2 | - | 미사용, 항상 ‘0’ |
| 1 | - | 미사용, 항상 ‘0’ |
| 0 | Input Voltage Error | 인가된 전압이 설정된 동작전압 범위를 벗어난 경우 |

### Realtime Tick (100)
장치의 시간을 나타내는 지수입니다.
| 단위      | 범위         | 상세 설명                         |
|-----------|--------------|------------------------------------|
| 1 [msec]  | 0 ~ 32,767   | 32,767 이후에는 ‘0’부터 다시 시작합니다. |


### Present Input Voltage (102)
현재 공급되는 전압입니다. 

### Status (106)
HX의 현재 상태를 나타냅니다.
| Bit | 명칭 | 상세 설명 |
|----|---|-----------|
| 7 | - | 미사용, 항상 ‘0’ |
| 6 | Bus Watchdog Error | Bus Watchdog Error가 발생한 경우 |
| 5 | - | 미사용, 항상 ‘0’ |
| 4 | - | 미사용, 항상 ‘0’ |
| 3 | - | 미사용, 항상 ‘0’ |
| 2 | - | 미사용, 항상 ‘0’ |
| 1 | - | 미사용, 항상 ‘0’ |
| 0 | Input Voltage Error | 인가된 전압이 설정된 동작전압 범위를 벗어난 경우 |



### Indirect Address(122~632), Indirect Data(634~889)
사용자는 이 기능을 이용해, 필요한 컨트롤 테이블을 모아서 이용할 수 있습니다.  
Indirect Address Table에 특정 주소를 세팅하면, Indirect Data Table은 특정 주소와 동일한 기능을 가지게 됩니다.  
예를 들어, Indirect Address 1(122)에 ‘65’을 쓰고, Indirect Data 1(634)에 ‘1’를 쓰게 되면, LED에 불이 들어옵니다. LED(65)의 값 또한 ‘1’로 쓰여있습니다.  
또한, LED(65)에 값을 쓰면, Indirect Data 1의 값 또한 똑같이 변합니다. Indirect Address에 특정 주소를 세팅하게 되면, Indirect Data는 그것과 동일한 테이블이 됩니다.  
주의해야 할 점은 2byte 이상의 길이를 가진 Control Table을 Indirect Address로 설정할 때입니다.  
Control Table Item의 모든 byte를 Indirect Address로 세팅 해주어야 정상 동작합니다.  
예를 들어, Indirect Data 2를 TableSync1 Read Address 1(1030)으로 사용하고 싶을 땐, 아래와 같이 세팅해야 합니다.

::: info example
예제 1 : 1 바이트 LED(65)를 Indirect Data 1(122)에 할당하기.  
Indirect Address 1(122) : LED의 주소 값인 ‘65’으로 변경.  
Indirect Data 1(634)을 ‘1’로 변경 : LED(65)값 또한 ‘1’로 변경되며 LED가 켜짐.  
Indirect Data 1(634)을 ‘0’로 변경 : LED(65)값 또한 ‘0’로 변경되며 LED가 꺼짐.
:::

::: info example
예제 2 : 2 바이트 길이의 TableSync1 Read Address 1(1030)를 Indirect Data 2(635)에 할당하기 위해서는 반드시 연속된 2 바이트를 모두 할당해야 함.  
Indirect Address 2(124) : 값을 TableSync1 Read Address 1의 첫번째 주소인 1030로 변경.  
Indirect Address 3(126) : 값을 TableSync1 Read Address 1의 두번째 주소인 1031로 변경.  
Indirect Data 2부터 3까지의 2바이트를 561(0x0231)로 변경 : TableSync1 Read Address 1(1030) 역시 561(0x0231)로 변경됨.

| Indirect Data 주소 | Goal Position 주소 | 저장된 HEX 값 |
|----|------|------|
| 635 | 1030 | 0x31 |
| 636 | 1031 | 0x02 |
:::

::: info
참고 : 2바이트 이상의 데이터를 Indirect Address에 할당하기 위해서는 모든 데이터의 주소를 ‘예제 2’와 같이 Indirect Address에 할당해주어야 합니다.
:::


### TableSync1 ID(1024~1029), TableSync2 ID(1222~1227), TableSync3 ID(1420~1425), TableSync4 ID(1618~1623), TableSync5 ID(1816~1821), TableSync1 Read Address(1030~1040), TableSync2 Read Address(1228~1238), TableSync3 Read Address(1426~1436), TableSync4 Read Address(1624~1634), TableSync5 Read Address(1822~1832), TableSync1 Read Size(1042~1052), TableSync2 Read Size(1240~1250), TableSync3 Read Size(1438~1448), TableSync4 Read Size(1636~1646), TableSync5 Read Size(1834~1844), TableSync1 Write Address(1054~1064), TableSync2 Write Address(1252~1262), TableSync3 Write Address(1450~1460), TableSync4 Write Address(1648~1658), TableSync5 Write Address(1846~1856), TableSync1 Write Size(1066~1076), TableSync2 Write Size(1264~1274), TableSync3 Write Size(1462~1472), TableSync4 Write Size(1660~1670), TableSync5 Write Size(1858~1868), TableSync1 Read Data(1078~1149), TableSync2 Read Data(1276~1347), TableSync3 Read Data(1474~1545), TableSync4 Read Data(1672~1743), TableSync5 Read Data(1870~1941), TableSync1 Write Data(1150~1221), TableSync2 Write Data(1348~1419), TableSync3 Write Data(1546~1617), TableSync4 Write Data(1744~1815), TableSync5 Write Data(1942~2013)  

사용자는 이 기능을 이용해, 다이나믹셀의 컨트롤 테이블 특정영역을 HX의 Table Sync Read Data와 Table Sync Write Data 영역에 매핑 하여 이용할 수 있습니다.  
HX의 TableSync는 총 5개 채널로 이루어져 있으며, 1번 채널은 엄지, 2번 채널은 검지, 3번 채널은 중지, 4번 채널은 약지, 5번 채널은 소지에 할당되어 있습니다.  
TableSync ID에는 매핑하고자 하는 다이나믹셀의 ID를 지정하고, TableSync Read Address와 TableSync Read Size를 이용하여 다이나믹셀로부터 읽을 시작 주소 및 크기를 지정하면 TableSync Read Data에 ID 설정 순서대로 데이터가 매핑됩니다. 또한 TableSync Write Address와 TableSync Write Size를 이용하여 다이나믹셀로 쓸 데이터의 시작 주소 및 크기를 지정하면 TableSync Write Data가 ID 설정 순서대로 데이터가 매핑됩니다. 설정 이후에 TableSync Enable을 1로 하면 TableSync 기능이 동작되어 TableSync Read Data는 상시로 데이터가 업데이트 되며, TableSync Write Data는 데이터를 쓸때 다이나믹셀의 맵핑된 컨트롤테이블이 업데이트 됩니다.  
TableSync ID의 값이 255 이면 해당 슬롯은 비활성됩니다. 각 채널 당 매핑할 수 있는 슬롯은 최대 6개 입니다.  
예를 들어, ID 1과 2로 설정된 다이나믹셀 X 2개의 Present Position과 Goal Position을 TableSync1 Read Data와, Table Sync1 Write Data에 매핑하기 위해 아래와 같이 설정합니다.

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
TableSync Enable을 1로 설정하면, TableSync1 Read Data 1~4는 ID1의 Present Position 값과 동일하며, Table Sync Read Data 5~8은 ID2의 Present Position 값과 동일하게 됩니다. 또한 Table Sync Write Data 1~4에 값을 쓰면 ID1의 Goal Position에 써지며, Table Sync Write Data 5~8에 값을 쓰면 ID2의 Goal Position에 써집니다.

::: info
참고 : 매핑하고자 하는 컨트롤테이블의 크기에 맞게 Table Sync Read Size와 Table Sync Write Size를 설정하세요. 크기를 잘못 설정할 경우 데이터가 깨질 수 있습니다.
:::



### Preset Index (2016)
Operating Mode(33)가 프리셋 동작 모드일 경우 저장되어 있는 3개의 프리셋 동작 중 어떠한 동작을 수행할 지 선택할 수 있습니다.  
프리셋 동작의 편집 및 저장은 DYNAMIXEL Wizard 2.0의 HX Hands 메뉴에서 가능합니다.

### Preset Motion Index (2017)
Preset Index(2016)에서 선택된 프리셋 동작의 시작과 끝 동작 사이의 선형 보간된 동작을 수행할 수 있는 기능입니다.  
0% 시 시작 동작, 100% 시 끝 동작을 수행합니다.

### Preset Motion Time (2018)
Preset Index(2016)에서 선택된 프리셋 동작의 재생 시 각 동작 재생 간 지연 시간 설정이 가능합니다.

</div>
