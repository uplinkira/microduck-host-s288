"""纯离线 MJCF 静态预检 CLI；源模型/资产只读，报告不得覆盖模型包文件。"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from model_preflight import inspect_model
from robot_profile import write_json


def main(argv=None):
    parser = argparse.ArgumentParser(description='只读检查 MJCF；不启动物理仿真、不访问串口')
    parser.add_argument('model', type=Path, help='模型包根 XML 路径')
    parser.add_argument('--output', type=Path, help='可选完整 JSON 报告；必须位于原模型包目录之外')
    args = parser.parse_args(argv)
    try:
        source = args.model.resolve()
        if args.output is not None:
            target = args.output.resolve()
            if target.is_relative_to(source.parent):
                raise ValueError('报告必须写到模型包目录之外，以保持原模型包只读')
        report = inspect_model(source)
        if args.output is not None:
            write_json(target, report)
        summary = {key: report.get(key) for key in ('path', 'status', 'ok', 'certified',
                   'training_ready', 'sha256', 'body_count', 'actuator_count', 'explicit_mass_sum')}
        summary.update(joint_count=len(report['joint_names']), asset_count=len(report['assets']),
                       errors=sum(c['level'] == 'error' for c in report['checks']),
                       warnings=sum(c['level'] == 'warning' for c in report['checks']),
                       checks=report['checks'], output=str(target) if args.output else None)
        print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
        return 0 if report['ok'] else 1
    except (OSError, ValueError) as exc:
        print(json.dumps({'status': 'failed', 'error': str(exc)}, ensure_ascii=False))
        return 2


if __name__ == '__main__':
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    raise SystemExit(main())
