import fs from 'node:fs/promises';
import path from 'node:path';
import assert from 'node:assert/strict';
import { SpreadsheetFile, Workbook } from '@oai/artifact-tool';

const [source, out] = process.argv.slice(2);
const { batch, fields, rows, summary } = JSON.parse(await fs.readFile(source, 'utf8'));
const wb = Workbook.create();
const main = wb.worksheets.add('工况汇总');
const raw = wb.worksheets.add('试次明细');
const numericMetrics = ['rmse_m','max_error_m','mean_speed_kmh','steering_rate_rms_deg_s'];
const groupLabels = ['跟踪 RMSE (m)','最大误差 (m)','实测均速 (km/h)','转向变化率 RMS (°/s)'];
const controllers = {pure_pursuit:'Pure Pursuit',stanley:'Stanley'};
const routes = {forward:'逆时针大环',clockwise:'顺时针小环'};

function line(sheet, address, value, bg, fg, size) {
  const range = sheet.getRange(address);
  range.merge();
  range.values = [[value]];
  range.format.fill = bg;
  range.format.font.color = fg;
  range.format.font.size = size;
  range.format.font.bold = size > 11;
}
function defaults(sheet, address, cols, header, widths) {
  const area = sheet.getRange(address);
  sheet.showGridLines = false;
  area.format.font.name = 'Microsoft YaHei';
  area.format.font.size = 11;
  area.format.font.color = '#182534';
  area.format.rowHeight = 20;
  area.format.verticalAlignment = 'center';
  sheet.getRange(header).format.fill = '#183951';
  sheet.getRange(header).format.font.color = '#FFFFFF';
  sheet.getRange(header).format.font.bold = true;
  for (let i = 0; i < cols; i++) {
    sheet.getCell(5, i).format.columnWidthPx = widths[i];
  }
  sheet.freezePanes.freezeRows(6);
}
const lastGroup = summary.length + 6;
const lastTrial = rows.length + 6;
defaults(main,`A1:N${lastGroup}`,14,'A5:N6',[135,112,126,62,70,72,78,78,77,77,84,84,86,86]);
line(main,'A1:N2',`Town04 路径跟踪 · ${summary.length} 组工况汇总`,'#102E44','#FFFFFF',19);
line(main,'A3:N3',`${rows.length} 次试验。连续指标按有效试次统计均值和样本标准差。`,'#E9F3F8','#244559',11);
line(main,'A4:N4','中段统计剔除路线首尾各 10 m。完成和越界分别按全程计数。','#E9F3F8','#244559',11);
main.getRange('A6:F6').values = [['路线','目标速度 (km/h)','算法','试次数','完成次数','越界次数']];
groupLabels.forEach((label, i) => {
  const col = 6 + i*2;
  const range = main.getRangeByIndexes(4,col,1,2);
  range.merge();
  range.values = [[label]];
  range.format.horizontalAlignment = 'center';
  main.getRangeByIndexes(5,col,1,2).values = [['均值','标准差']];
});
main.getRange(`D7:F${lastGroup}`).setNumberFormat('0');
main.getRange(`G7:N${lastGroup}`).setNumberFormat('0.000');
main.getRange(`B7:B${lastGroup}`).setNumberFormat('0');
main.getRange(`B7:B${lastGroup}`).format.horizontalAlignment = 'center';
for (const col of ['H','J','L','N']) main.getRange(`${col}7:${col}${lastGroup}`).setNumberFormat('0.0000');

defaults(raw,`A1:L${lastTrial}`,12,'A6:L6',[112,124,108,62,80,80,91,113,110,114,125,166]);
line(raw,'A1:L2',`Town04 路径跟踪 · ${rows.length} 次试验明细`,'#102E44','#FFFFFF',19);
line(raw,'A3:L3',`数据来源  ${batch}/summary.csv`,'#E9F3F8','#244559',11);
line(raw,'A4:L4','完成和越界字段中，1 表示是，0 表示否。','#E9F3F8','#244559',11);
raw.getRange('A6:L6').values = [['路线代码','算法代码','目标速度 (km/h)','重复号','完成','越界','耗时 (s)','终点距离 (m)','RMSE (m)','最大误差 (m)','实测均速 (km/h)','转向变化率 RMS (°/s)']];
raw.getRange(`A7:L${lastTrial}`).values = rows;
raw.getRange(`G7:L${lastTrial}`).setNumberFormat('0.000');

const expected = [];
summary.forEach((group,i) => {
  const lo = 7 + group.start, hi = lo + group.count - 1, row = 7 + i;
  const refs = (key) => {
    const col = String.fromCharCode(65 + fields.indexOf(key));
    return `'试次明细'!${col}${lo}:${col}${hi}`;
  };
  main.getRange(`A${row}:C${row}`).values = [[routes[group.case],group.speed,controllers[group.controller]]];
  main.getRange(`D${row}:N${row}`).formulas = [[
    `=COUNT(${refs('repeat')})`, `=SUM(${refs('completed')})`, `=SUM(${refs('outside_lane')})`,
    ...numericMetrics.flatMap(key => {
      const input = refs(key);
      return [`=IF(COUNT(${input})=0,"",AVERAGE(${input}))`,
              `=IF(COUNT(${input})<2,"",STDEV(${input}))`];
    })
  ]];
  if (i % 4 < 2) main.getRange(`A${row}:N${row}`).format.fill = '#EDF5F9';
  if (group.outside) main.getRange(`F${row}`).format.fill = '#F9DED0';
  if (i % 2 === 0) raw.getRange(`A${lo}:L${hi}`).format.fill = '#EDF5F9';
  expected.push([group.count,group.complete,group.outside,...numericMetrics.flatMap(key => group.metrics[key])]);
});
wb.recalculate();
const actual = main.getRange(`D7:N${lastGroup}`).values;
for (let i=0;i<expected.length;i++) for (let j=0;j<expected[i].length;j++) {
  const value = actual[i][j], target = expected[i][j];
  assert.ok(target === null ? value === '' || value === null
                            : typeof value === 'number' && Math.abs(target - value) < 1e-8,
            `Unexpected summary value at group ${i + 1}, column ${j + 4}: ${value}`);
}
const xlsx = await SpreadsheetFile.exportXlsx(wb);
await xlsx.save(path.join(out, 'Town04_实验结果汇总.xlsx'));
const preview = await wb.render({sheetName:'工况汇总',range:`A1:N${lastGroup}`,scale:1,format:'png'});
await fs.writeFile(path.join(out, 'workbook-preview.png'), new Uint8Array(await preview.arrayBuffer()));
