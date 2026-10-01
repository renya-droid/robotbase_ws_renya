"""Robot ROS 2 Launch Manager alias parser."""

import re
import shlex
from pathlib import Path

from robot_config import (
    ENV_ALIAS,
    SIM_ENV_ALIAS,
    SIM_SOURCE_ALIAS,
    SOURCE_ALIAS,
)


def parse_bash_aliases(alias_file_path):
    """bash_alias2ファイルからエイリアスとプリセットを解析（複数行対応）"""
    groups = {}
    presets = []
    current_group = "その他"
    current_description = ""
    current_preset_name = None
    
    try:
        with open(alias_file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        lines = content.split('\n')
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            
            if not line:
                current_description = ""
                i += 1
                continue
            
            # プリセット名の検出
            if line.startswith('# PRESET:'):
                current_preset_name = line.replace('# PRESET:', '').strip()
                i += 1
                continue
            
            # プリセットアイテムの検出
            if line.startswith('# PRESET_ITEMS:') and current_preset_name:
                items_str = line.replace('# PRESET_ITEMS:', '').strip()
                items = [item.strip() for item in items_str.split(',')]
                presets.append((current_preset_name, items))
                current_preset_name = None
                i += 1
                continue
            
            if line.startswith('# GROUP:'):
                current_group = line.replace('# GROUP:', '').strip()
                if current_group not in groups:
                    groups[current_group] = []
                current_description = ""
                i += 1
                continue
            
            if line.startswith('#') and not line.startswith('# GROUP:'):
                current_description = line.lstrip('#').strip()
                i += 1
                continue
            
            if line.startswith('alias '):
                # 複数行のエイリアスを結合
                full_line = line
                while full_line.endswith('\\') and i + 1 < len(lines):
                    i += 1
                    full_line = full_line[:-1] + ' ' + lines[i].strip()
                
                # エイリアスをパース
                match = re.match(r"alias\s+([^=]+)='(.+)'", full_line, re.DOTALL)
                if match:
                    alias_name = match.group(1).strip()
                    command = match.group(2).strip()
                    
                    if alias_name.endswith('install_packages') or alias_name.endswith('launcher'):
                        i += 1
                        continue

                    # The GUI is independent of ~/.bashrc. Expand real and
                    # simulation helper aliases into explicit commands.
                    ws_dir = Path(alias_file_path).resolve().parent.parent
                    setup_bash = ws_dir / 'install' / 'setup.bash'
                    activate_script = ws_dir / 'bash' / 'activate_koko_env.sh'
                    activate_sim_script = ws_dir / 'bash' / 'activate_koko_sim_env.sh'
                    env_command = f'source {shlex.quote(str(activate_script))}'
                    sim_env_command = f'source {shlex.quote(str(activate_sim_script))}'
                    source_command = (
                        f'{env_command} && cd {shlex.quote(str(ws_dir))} && '
                        f'source {shlex.quote(str(setup_bash))}'
                    )
                    sim_source_command = (
                        f'{sim_env_command} && cd {shlex.quote(str(ws_dir))} && '
                        f'source {shlex.quote(str(setup_bash))}'
                    )
                    if command.startswith(f'{SIM_SOURCE_ALIAS} && '):
                        command = command.replace(
                            f'{SIM_SOURCE_ALIAS} && ', f'{sim_source_command} && ', 1)
                    elif command.startswith(f'{SIM_ENV_ALIAS} && '):
                        command = command.replace(
                            f'{SIM_ENV_ALIAS} && ', f'{sim_env_command} && ', 1)
                    elif command.startswith(f'{SOURCE_ALIAS} && '):
                        command = command.replace(
                            f'{SOURCE_ALIAS} && ', f'{source_command} && ', 1)
                    elif command.startswith(f'{ENV_ALIAS} && '):
                        command = command.replace(
                            f'{ENV_ALIAS} && ', f'{env_command} && ', 1)
                    
                    description = current_description if current_description else alias_name
                    
                    if current_group not in groups:
                        groups[current_group] = []
                    groups[current_group].append((alias_name, command, description))
                    
                    current_description = ""
            
            i += 1
    
    except Exception as e:
        print(f"エイリアスファイルの読み込みエラー: {e}")
    
    return groups, presets
