#!/usr/bin/env python3
"""Agente conversacional multimodal: texto/imagen -> habilidad -> resultado."""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

if __package__:
    from .clients import HttpFailure, HttpSkillClient, OllamaModel
    from .multimodal_agent import ConversationMemory, MultimodalAgent, load_config
else:
    from clients import HttpFailure, HttpSkillClient, OllamaModel
    from multimodal_agent import ConversationMemory, MultimodalAgent, load_config


def main():
    parser = argparse.ArgumentParser(description='Pepper: conversación, visión y habilidades')
    parser.add_argument('text', nargs='?', help='Sin texto inicia una conversación interactiva')
    parser.add_argument('--skill', help='Probar directamente una habilidad, sin IA')
    parser.add_argument('--agent-config', default=str(
        Path(__file__).with_name('agent_config.json')))
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--memory-file', default=str(
        Path(os.getenv('XDG_STATE_HOME', str(Path.home() / '.local/state')))
        / 'pepper_agent/memory.json'))
    group.add_argument('--no-memory', action='store_true', help='Memoria solo durante esta sesión')
    parser.add_argument('--watch', action='store_true',
                        help='Observar y aplicar reglas automáticamente')
    parser.add_argument('--interval', type=float, default=5.,
                        help='Pausa real entre observaciones')
    parser.add_argument('--iterations', type=int, default=0, help='0 observa hasta Ctrl+C')
    args = parser.parse_args()
    if args.watch and (args.text or args.skill):
        parser.error('--watch se usa sin texto ni --skill')
    if not math.isfinite(args.interval) or not 0 < args.interval <= 60 or args.iterations < 0:
        parser.error('interval debe estar entre 0 y 60; iterations debe ser >= 0')
    try:
        bridge = HttpSkillClient(os.getenv('SKILL_BRIDGE_URL', 'http://127.0.0.1:8765'))
        if args.skill:
            if args.skill not in {s['name'] for s in bridge.discover()}:
                raise ValueError('Habilidad no disponible')
            result = bridge.execute(args.skill)
            if 'observation' in result:
                result['observation'].pop('image_base64', None)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0 if result.get('ok') else 1
        config = load_config(args.agent_config)
        model = OllamaModel(config['model'],
                            os.getenv('OLLAMA_URL', 'http://127.0.0.1:11434/api/chat'),
                            thinking=config.get('thinking', False))
        memory = ConversationMemory(None if args.no_memory else args.memory_file,
                                    config['max_history_turns'])
        agent = MultimodalAgent(bridge, model, config, memory)
        if args.watch:
            count = 0
            while not args.iterations or count < args.iterations:
                result = agent.turn('Observa la escena actual y aplica las reglas disponibles.',
                                    watch=True)
                print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
                count += 1
                if not args.iterations or count < args.iterations:
                    time.sleep(args.interval)
        elif args.text:
            result = agent.turn(args.text)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 1 if result.get('result', {}).get('ok') is False else 0
        else:
            print('Pepper listo. /reset borra la memoria; /exit termina la conversación.')
            while True:
                text = input('Tú > ').strip()
                if text == '/exit':
                    break
                if text == '/reset':
                    memory.clear()
                elif text:
                    result = agent.turn(text)
                    print('Pepper > ' + result['reply'])
                    if result.get('result'):
                        print('[Habilidad: ' + str(result['result'].get('status')) + ']')
        return 0
    except (KeyboardInterrupt, EOFError):
        return 0
    except HttpFailure as exc:
        print(json.dumps(exc.result, ensure_ascii=False, indent=2), file=sys.stderr)
        return 1
    except (RuntimeError, ValueError, KeyError, OSError) as exc:
        print(f'[ERROR] {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
