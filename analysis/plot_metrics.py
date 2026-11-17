from pathlib import Path
import json
import sys
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import fontManager
from audit_panel_alignment import require_matplotlib_panel_alignment

source, output = map(Path, sys.argv[1:])
data = json.loads(source.read_text(encoding='utf-8'))['summary']
font_path = '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
fontManager.addfont(font_path)
mpl.rcParams.update({
    'font.family': ['DejaVu Sans', 'Droid Sans Fallback', 'sans-serif'],
    'font.size': 9,
    'axes.spines.top': False, 'axes.spines.right': False,
    'axes.linewidth': .65, 'axes.axisbelow': True, 'axes.unicode_minus': False,
    'pdf.fonttype': 42, 'svg.fonttype': 'none', 'legend.frameon': False,
    'savefig.facecolor': 'white',
})
fig, axes = plt.subplots(2, 3, figsize=(9.5,5.7))
fig.subplots_adjust(left=.085,right=.974,bottom=.15,top=.83,wspace=.38,hspace=.68)
metrics = [
    ('rmse_m','跟踪 RMSE (m)'),
    ('mean_speed_kmh','实测平均速度 (km/h)'),
    ('steering_rate_rms_deg_s','转向变化率 RMS (°/s)'),
]
styles = [
    ('pure_pursuit','Pure Pursuit','#176487','o',-.022),
    ('stanley','Stanley','#AE4A20','s',.022),
]
for row,(case,routename) in enumerate([('forward','逆时针大环'),('clockwise','顺时针小环')]):
    route_rows=[item for item in data if item['case']==case]
    speeds=sorted({item['speed'] for item in route_rows})
    for col, (key, label) in enumerate(metrics):
        ax=axes[row,col]
        ax.set_title(f'{chr(97+row*3+col)}   {routename}  /  {label}',loc='left',fontsize=9.3,pad=7)
        ax.set_xticks(speeds)
        ax.grid(axis='y',color='#E1E6EB',lw=.6)
        if not speeds:
            ax.text(.5,.5,'暂无试次',transform=ax.transAxes,ha='center',va='center',color='#55616A')
            ax.set_yticks([])
            continue
        span=max(1, speeds[-1]-speeds[0])
        ax.set_xlim(speeds[0]-span*.11,speeds[-1]+span*.11)
        if key=='mean_speed_kmh':
            ax.plot(speeds,speeds,ls='--',color='#92999E',lw=.9,zorder=1,label='实测等于目标')
        for controller,title,color,marker,delta in styles:
            entries=sorted((item for item in route_rows if item['controller']==controller),key=lambda item:item['speed'])
            if not entries:
                continue
            y = [float('nan') if item['metrics'][key][0] is None else item['metrics'][key][0]
                 for item in entries]
            std = [item['metrics'][key][1] or 0 for item in entries]
            # Offset points slightly so two almost identical means remain visible.
            ax.errorbar([item['speed']+delta*span for item in entries],y,yerr=std,marker=marker,color=color,
                        markeredgecolor='white',markeredgewidth=.4,ms=5,ls='-' if marker=='o' else ':',
                        elinewidth=1.15,lw=1.3,capsize=2,label=title,zorder=3)
        if row==1:
            ax.set_xlabel('目标速度 (km/h)')
        ax.tick_params(length=3,width=.7)
        ax.margins(y=.11)
        ax.set_ylim(bottom=0)
legend = {}
for ax in axes.flat:
    handles, labels = ax.get_legend_handles_labels()
    legend.update(zip(labels, handles))
labels = [label for label in ('Pure Pursuit', 'Stanley', '实测等于目标') if label in legend]
fig.legend([legend[label] for label in labels],labels,loc='upper center',bbox_to_anchor=(.57,.928),ncol=3)
fig.text(.085,.962,'Town04 两种路径跟踪算法的批量实验结果',fontsize=14,weight='bold',va='top')
fig.text(.085,.035,'点为有效试次的均值，误差棒为样本标准差（至少 2 个值）。各面板纵轴独立设置。',fontsize=8)
require_matplotlib_panel_alignment(fig,json_out=output/'plot.alignment.json',strict=True)
fig.savefig(output/'Town04_三项指标对比.pdf')
fig.savefig(output/'Town04_三项指标对比.svg')
fig.savefig(output/'Town04_三项指标对比.png',dpi=300)
