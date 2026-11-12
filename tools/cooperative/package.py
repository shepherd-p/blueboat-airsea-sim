"""Assemble review videos, compact evidence and an audited public source ZIP."""
from pathlib import Path
import argparse,datetime,hashlib,json,shutil,subprocess,zipfile

ROOT=Path(__file__).resolve().parents[2]


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def display(path):
    value=path.as_posix()
    return value[5].upper()+':/'+value[7:] if value.startswith('/mnt/') and value[6:7]=='/' else value


def package(run,video,report,out,inspection=None):
    date=datetime.date.today().isoformat()
    validation=json.loads((run/'cooperative_validation.json').read_text());assert validation['passed']
    checks=json.loads((video/'video_checks.json').read_text());assert checks['passed']
    closeup_checks=json.loads((video/'clean_closeup_checks.json').read_text());assert closeup_checks['passed']
    out.mkdir(parents=True,exist_ok=False)
    folders={n:out/n for n in ('01_Videos','02_Verification','03_Source')}
    for p in folders.values():p.mkdir()
    for name in ('Mutual_Observation_And_Speed_Coordination.mp4','Both_Cameras_Clean.mp4','Both_Cameras_Closeup_Clean.mp4'):
        shutil.copy2(video/name,folders['01_Videos']/name)
    for path in (video/'Mutual_Observation_Preview.jpg',video/'video_checks.json',video/'clean_closeup_checks.json',
                 video/'frame_evidence.json',video/'clean_closeup_frame_evidence.json',
                 run/'cooperative_validation.json',run/'session_manifest.json',run/'source_hashes.json',
                 run/'native_library_hashes.json',run/'input_hashes.json',
                 run/'ardurover_used.parm',run/'arduplane_used.parm',run/'cooperative_config.json',
                 run/'profile.json',
                 run/'aircraft_skin_tags.json',
                 report/'Flight_Paths.png',report/'Coordination_And_Visual_Errors.png',report/'validation_summary.json'):
        shutil.copy2(path,folders['02_Verification']/path.name)
    if inspection is not None:
        assert json.loads((inspection/'inspection_checks.json').read_text())['passed']
        skin=folders['02_Verification']/'Wing_Skin_Installation';skin.mkdir()
        for name in ('06_wing_underside.png','07_underside_oblique.png','02_side.png','04_top.png','inspection_checks.json','aircraft_skin_tags.json'):
            shutil.copy2(inspection/name,skin/name)
    rows=[json.loads(s) for s in (run/'independent_perception.jsonl').read_text().splitlines()]
    examples=folders['02_Verification']/'Original_Camera_Examples';examples.mkdir();records=[]
    for kind in ('air','boat'):
        tagged=[r for r in rows if r[kind]['tag_detected']]
        shaped=[r for r in rows if r[kind]['measurement'] and not r[kind]['tag_detected']]
        selected=[('tag',tagged[len(tagged)//2]),('shape_only',shaped[len(shaped)//2])]
        for mode,r in selected:
            name=f'{kind}_{mode}_{r["index"]:06d}.jpg'
            shutil.copy2(run/f'frames/{kind}/{r["index"]:06d}.jpg',examples/name)
            records.append(dict(file=name,index=r['index'],t=r['t'],kind=kind,result=r[kind],sha256=digest(examples/name)))
    (examples/'provenance.json').write_text(json.dumps(records,indent=2)+'\n')
    files=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=ROOT).decode().split('\0')
    excluded=('results/','build/','install/','delivery/','.local_assets/','docs/ISSUES_AIRSEA_CN.md')
    files=sorted(set(n for n in files if n and not n.startswith(excluded) and n!='SOURCE_MANIFEST.json' and (ROOT/n).is_file()))
    forbidden={'.stl','.zip','.mp4','.so','.exe','.dll','.bin','.pem','.key','.pyc'}
    assert not any(Path(n).suffix.lower() in forbidden or Path(n).name.startswith('.env') for n in files)
    manifest=dict(date=date,scope='Public source for wing-skin AprilTags, mutual visual observation and classical speed coordination. No manufacturer CAD, firmware/runtime binaries, private correspondence, logs or videos.',
                  hash_basis='Exact uncompressed ZIP entry bytes, excluding this manifest.',sha256={n:digest(ROOT/n) for n in files})
    source=folders['03_Source']/'BlueBoat_AirSea_Cooperative_Source.zip'
    with zipfile.ZipFile(source,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for n in files:z.write(ROOT/n,'BlueBoatAirSeaLab/'+n)
        z.writestr('BlueBoatAirSeaLab/SOURCE_MANIFEST.json',json.dumps(manifest,indent=2)+'\n')
    with zipfile.ZipFile(source) as z:
        assert z.testzip() is None
        assert all(hashlib.sha256(z.read('BlueBoatAirSeaLab/'+n)).hexdigest()==h for n,h in manifest['sha256'].items())
    a=validation['observation']['air'];b=validation['observation']['boat']
    count=sum(validation['checks'].values())
    text=f'''# {date} 翼面贴合标记与机艇双向视觉

本次完成的是教授新增五项要求的受限仿真基线，使用原来的 BlueBoat 模型、无外露螺旋桨 EDF 固定翼、Gazebo Sea State 1、真实 ArduPlane/ArduRover SITL。
飞机原先突出的标记板已删除，改为左右主翼下表面的 AprilTag 涂装；编码区 20 厘米，含白边 25 厘米。没有新增板件或抬高的贴片网格。机翼外形保持原样，并修正了旧翼面朝向。

## 会议展示

- `01_Videos/Mutual_Observation_And_Speed_Coordination.mp4`：两路真实相机、同帧放大、相对 ENU 位置/速度、双方指令和实际速度。
- `01_Videos/Both_Cameras_Closeup_Clean.mp4`：推荐无文字展示版，左右分别是船和飞机的同帧数字放大，无检测框。
- `01_Videos/Both_Cameras_Clean.mp4`：并排原始相机画面，无文字和检测框。
- `02_Verification/Flight_Paths.png`：实际飞行、船舶运动与任务航点参考。
- `02_Verification/Wing_Skin_Installation/`：实际 Gazebo 正下方、斜下方、侧面和上方视图，以及外形保持与解码校验。
- `03_Source/BlueBoat_AirSea_Cooperative_Source.zip`：整理后的开源源码，已逐文件校验；不含厂商 CAD 或固件二进制。

## 本轮实际结果

- 协同区间 {validation['cooperation_duration_s']:.3f} 秒；本地工程验收 {count}/{len(validation['checks'])} 项通过。这些阈值是工程检查，并非教授另行规定的正式验收标准。
- 飞机看船：有效视觉覆盖 {100*a['visual_measurement_time_fraction']:.2f}%，最长测量间隔 {a['maximum_measurement_gap_s']:.3f} 秒，位置误差 P95 {a['position_error_m']['p95']:.3f} 米。
- 船看飞机：有效视觉覆盖 {100*b['visual_measurement_time_fraction']:.2f}%，最长测量间隔 {b['maximum_measurement_gap_s']:.3f} 秒，位置误差 P95 {b['position_error_m']['p95']:.3f} 米；目标速度误差 P95 {b['velocity_error_mps']['p95']:.3f} 米/秒。
- 两个方向分别记录到 {a['tag_frames']} / {b['tag_frames']} 帧 AprilTag；远处无标签的图像继续使用轮廓估计。缺测与预测均单独标记。
- 自主起飞、协调巡航、返航、跑道接地、停止与自动解除武装均有物理状态和飞控事件验证；接地下沉速度 {validation['touchdown_sink_mps']:.3f} 米/秒。
- 真实相机平均采样 {a['actual_sensor_mean_hz']:.2f} Hz。30fps 视频会重复原始采样帧，没有光流插帧。

## 复现和证据

本机源码：`{display(ROOT)}`。在该项目目录的 PowerShell 中运行 `./RunCooperative.ps1` 可启动新试验，结束后自动执行独立验收。
公开源码的安装、原始船体导入及运行说明见 `README.md`、`docs/COOPERATIVE_20261112.md` 和 `docs/SKIN_TAGS_20261113.md`；导入厂商模型后才会演示该船，素材丢失会报错。
本轮完整原始 JPEG、逐帧估计、全部物理日志和冻结源码：`{display(run)}`。本文件夹的证据仅为便于会议查阅的副本。

## 应汇报的边界和下一步

这是已知机型/船型、均匀天空、理想相机和有限海况下的经典视觉基线。获取或丢失目标时允许通过双方 EKF 通信重新指向，飞机 IMU 姿态参与轮廓尺度修正和曲面标记定位；因此不能称为无 GNSS、无通信的纯视觉系统。
翼面涂装保持模型外形，真实涂层或贴膜的厚度、粗糙度、粘接、结构间隙及气动影响仍需实机工程验证，当前通用飞行动力学不能证明气动影响为零。
双向视觉验收针对协同巡航区间，不包含起飞和降落的全程可见性。短时缺测仍如实统计。
下一步应做独立重复试验、固定速度对照、仅轮廓/仅标签消融，以及云层、其他目标、噪声、模糊和真实图像测试；尚未完成真实机艇参数标定、实物验证、RL 或论文实验。
'''
    (out/'START_HERE_说明.md').write_text(text,encoding='utf-8')
    issue='''# 本轮问题与处理

1. 飞机自身蓝色部件误当作船：下视候选加入水面投影、物理尺寸、运动关联；相机安装到较低的机鼻位置。回归检查覆盖蓝色前景干扰。
2. 灰色机腹被亮度阈值切碎：改为相对于当前天空背景提取完整飞机轮廓，保留阴影和深色机尾。
3. 轮廓测距较准但转弯滤波拖后：修正径向测量协方差，采用对不规则采样时间一致的连续加速度过程噪声。
4. 目标丢失与图像延迟：短时滤波预测用于相机指向，但不算检测；重新捕获使用明确记录的 EKF 通信提示，控制器使用含处理延迟的图像年龄。
5. 结束时偶有最后一对图片未被估计：在渲染进程退出后读取剩余图像记录，保证原始图片与估计一一对应。
6. 11 月 12 日早期失败试验完整保留。当天第三轮船载定位 P95 38.206 米、最长记录年龄 2.580 秒，未作为最终结果交付。最终结果仅以本包 validation_summary.json 对应的试验为准。
7. 速度曲线揭示 ArduRover 的旧 WP_SPEED=0.8 把更高船速指令截断：为协同任务单独生成参数文件，使 WP_SPEED 与协调器最大船速一致；原有基础演示参数保留。最终补跑验证实际船速在初始加速后仍随指令变化。
8. 飞机独立标记板不合理：已删除，替换为原有主翼下表面的贴合涂装，保持几何外形。修正旧机翼内向法线及下表面纹理镜像；曲面实际角点与通信 IMU 姿态共同估计飞机中心位置。20 厘米标记搭配船载 14 度水平视场，代替原来 28 厘米标记与 20 度视场；不承诺远距离一直能识别标签。
9. 本轮第一版发现纹理镜像后主动停止；第二版暴露接近阶段的旧视觉关联门限问题，改为结合滤波位置协方差进行关联，原始图像回放确认能够恢复。失败日志保存在原始 results 目录；正式交付只使用通过独立验收的新试验。贴合涂装降低突起板件造成的风险，但通用 SITL 无法验证真实贴膜厚度、粗糙度、粘接和气动效应。
10. 第三次启动在 WSL D3D12/OpenGL 上下文初始化时失败，没有生成相机图像；清理后使用相同仿真代码与参数重新运行。该次失败日志保存在 cooperative_skin_20261113_03，不能作为飞行结果。

未完成事项与研究局限见 START_HERE_说明.md。
'''
    (out/'问题与局限记录.md').write_text(issue,encoding='utf-8')
    (out/'DELIVERY_SHA256.json').write_text(json.dumps({p.relative_to(out).as_posix():digest(p) for p in sorted(out.rglob('*')) if p.is_file()},indent=2)+'\n')
    print('Delivery:',out,'source files:',len(files),'source ZIP:',source.stat().st_size)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for n in ('run','video','report','out'):p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--inspection',type=Path)
    a=p.parse_args();package(a.run,a.video,a.report,a.out,a.inspection)
