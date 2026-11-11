"""生成已纠错的外发 XLSX，不改同事原始工作簿；需要 openpyxl，仅文档工具使用。"""
import argparse
from pathlib import Path
from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill, Alignment
from openpyxl.utils import get_column_letter


def export(path):
    book=Workbook(); book.remove(book.active)
    sheets={
        '实测需求总览':[
            ['数据项','用途/阶段','上位机v1_04能力','前置或剩余工作','交付文件'],
            ['ID与实际语义','实机映射；仿真接入可并行','映射实测表，人工填写导出','逐台小幅电机正向，不能在direction未知时声称模型正向','joint_measurements.csv及同名.csv.json'],
            ['direction与模型q_home','部署前必须确认','实测表允许空白；不预填默认+1或旧STAND','方向按模型轴实测；q_home由修正后的模型决定，嘴部不入行走策略','实测表及标定姿态照片'],
            ['电脑端零位','坐标变换与重启恢复核验','设零位、保存、导出零位JSON','不是电机Flash；q_home不是ex_zero；换装配重新核对','calib_host_xianduck.json'],
            ['纯位置阶跃','执行器控制验证','0→小幅目标→0，速度/扭矩前馈为0；输出端Kp/Kd可选最多三档','先确认低增益台架；±5°为试验幅度，不是普通上位机角度限位','measurement/会话/samples.csv、trial_NN.json'],
            ['单台及整轮往返','控制循环时序预算','单台1000次；13个非嘴ID每轮串行各一笔，共1000轮；有效率/均值/P99/最大值','主机API到有效反馈非机械响应；统计含丢帧；最终运行平台必须复测','session.json、samples.csv、samples.jsonl'],
            ['摆锤物理量','辨识拟合前必须提供','必填配重kg、均匀杆kg、长度m和台架说明','均匀杆长度须与转轴至配重重心长度假设一致；非均匀/有限配重需补质心惯量','session.json.config'],
            ['三轨迹×三增益','执行器辨识输入','三条6秒轨迹×自选最多三档输出端Kp；可形成9组，正常组间慢回零','不默认50/500/2000；增益未确认不运行；S288拟合器需算法适配','trial_01.json至trial_09.json'],
            ['IMU/质量/接触','完整训练与部署','不能由舵机串口自动获得','IMU安装变换/噪声/时延，模型质量惯量单位、碰撞和站姿仍需确认','传感器日志、动力学核验及任务契约']
        ],
        '微动记录模板':[['电机ID','预期语义（待实测）','操作','实际语义','direction(+1或-1)','模型q_home(rad)','备注','确认人']],
        '设零位步骤':[
            ['步骤','操作','验收/边界'],
            ['准备','架空固定、观察夹点、低速度/低增益单台微动','默认无机械角度限位，不进行整机自动扫描运动'],
            ['映射','先确认实际关节与模型轴，再填direction','未知必须空白，不把默认+1当已确认'],
            ['站姿','算法确定当前模型q_home，实机摆同一参考姿态并拍照','禁止使用旧表错位STAND；初始模型home全零不等于稳定站姿'],
            ['设零','卸力静止后逐台设零位，保存电脑端零位','每次静止三次读数；不是写电机Flash'],
            ['交付','导出零位JSON及映射表','q_model=q_home+direction*radians(host_deg)，仅在同姿态标零成立'],
            ['复核','低速小幅点动后回零，断电重连再核验','记录偏差，不宣称软件测试已证明实机重启零位正确']
        ],
        '运行时数据':[
            ['项目','采集方法','统计与边界'],
            ['纯位置跟踪','选择单台纯位置阶跃；默认3°、5秒、50Hz，Kp3/Kd0.3是起始软件值非通用已验收参数','保存实际绝对位置、相对角、量化后线上值、零速度前馈、温压扭矩、有效帧；输出RMSE，机械滞后需另外分析'],
            ['单台往返','单台ID，默认1000次，零增益请求','不是ACK；标明主机、系统、USB板、波特率、固件。均值<1ms/P99<2ms不是源码保证，可另行商定目标'],
            ['整轮往返','全部13非嘴ID，共1000轮，串行请求','每轮有效数量必须13；统计含采集记盘开销的主机轮耗时，最终部署环境复测；50Hz总预算20ms还含推理/IMU'],
            ['波特率','Windows设备/驱动及真实帧校验；部署Linux另做termios验收','stty输出不是实际线速证据；UART回环只检串口通路，不能证明舵机响应或13台预算'],
            ['停止保护','停止按钮、Esc、关闭请求；主动试验首个坏反馈/故障/温压/力矩/跟踪异常即退出','取消或故障不回零，三次零增益；无反馈则不能保证已卸力。默认壳温70℃/反馈扭矩0.6N·m为试验停止阈值，不是持续额定扭矩或硬实时扭矩上限']
        ],
        '摆锤辨识':[
            ['项','要求','说明'],
            ['配重质量mass','kg，实称','±1g可作为测量目标，记录仪器精度'],
            ['杆质量arm_mass','kg，实称','均匀杆假设；BAM读取arm_mass，下游不应依赖某旧脚本arm-mass拼写'],
            ['length','m，转轴至配重重心','若杆长度与该长度不同，不能只用三参数均匀杆模型；需要实际杆质心/惯量'],
            ['台架零位和direction','竖直向下为0；与重力模型一致的正向','不能使用整机STAND零位直接作为摆锤零位；先人工摆正并标定'],
            ['sin_time_square','6秒；sin(t²)','最高约±57.3°；不经过普通位姿限速，单台固定台架'],
            ['up_and_down','6秒；0→π/2→0.8π/2，三次平滑','到达约90°，检查无碰撞'],
            ['lift_and_drop','6秒；2秒到-π/2，此后Kp=Kd=0','零增益自由下落；不把保留阻尼的普通卸力冒称自由落体'],
            ['九组','选择全部轨迹+三个逗号分隔的输出端Kp','先用低增益单组联调；不默认XL330的50/500/2000；正常组后10°/s回零，取消/异常不回零'],
            ['输出','每组trial_NN.json+整会话session.json/samples.csv/jsonl','包含mass/arm_mass/length、timestamp/position/speed/goal_position/torque_enable；无PWM load，不伪造'],
            ['拟合','需要S288专用控制与动力学参数适配','BAM现有unitree模块为Go1，不能原样使用23.7N·m等参数；本机JSON明确bam_compatible=false'],
            ['抖动','对各trial相邻timestamp作差；session含间隔std/P99','可把trial文件单独放到文件夹用bam.jitter，勿混入session.json；本轮只验证字段和统计，不运行拟合']
        ],
        '版本与有效性':[
            ['版本','说明'],['v1_04','标题：宇树s288舵机microduck关节运动上位机；EXE：unitree_s288_microduck_joint_ui_v1_04.exe'],
            ['资料核验','本地Microduck RL 53b8971 / runtime 1fa3ff7 / BAM 620a64f；不声称已拉取最新远端'],
            ['原表STAND','原表头颈与右腿数值错位且包含本机无head_roll，本工作簿不保留错误角度；q_home全部留待模型确认'],
            ['验证层级','软件离线/假总线测试不等于实机方向、零位、时延或执行器拟合通过'],
            ['未交付工具','原表tools/s288_mapping.py convert在本次可用工程中未找到，不作为可执行命令'],
            ['安全','软件反馈阈值不能替代外部电源限流、机械固定及人工监护；默认仿真不生成实测数据']
        ]
    }
    names=['left_hip_yaw','left_hip_roll','left_hip_pitch','left_knee','left_ankle',
           'right_hip_yaw','right_hip_roll','right_hip_pitch','right_knee','right_ankle','neck_pitch','head_pitch','head_yaw','mouth']
    ids=[9,10,11,12,13,1,2,3,4,5,6,7,8,14]
    for mid,name in zip(ids,names):
        sheets['微动记录模板'].append([mid,name,'电机输出正向小幅点动，观察后回零','','','','嘴不进策略；仍须闭合姿态和实际正向说明' if mid==14 else '','',''][:8])
    for title,rows in sheets.items():
        s=book.create_sheet(title)
        for row in rows:s.append(row)
        s.freeze_panes='A2'; s.auto_filter.ref=s.dimensions
        for cell in s[1]:cell.font=Font(bold=True,color='FFFFFF');cell.fill=PatternFill('solid',fgColor='24486B')
        for row in s:
            for cell in row:cell.alignment=Alignment(wrap_text=True,vertical='top')
        for col in range(1,s.max_column+1):s.column_dimensions[get_column_letter(col)].width=24 if col==1 else 48
        for i in range(2,s.max_row+1):s.row_dimensions[i].height=62
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True); book.save(path)
    return path

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('output');args=ap.parse_args(); print(export(args.output))
