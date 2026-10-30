"""Offline labels for the client's region observations, never guessed heights."""
import json


class RegionContext:
    def __init__(self, root):
        self.root, self.data = root, None

    def describe(self, sample, build):
        if self.data is None:
            self.data = {}
            path = self.root/'region-context.json' if self.root else None
            if path and path.exists():
                try:
                    value = json.loads(path.read_text(encoding='utf-8'))
                    if value.get('schema') == 1 and value.get('kind') == 'forever-region-context' and value.get('build') == build:
                        self.data = value
                except (ValueError, OSError):
                    pass
        names = self.data.get('labels', {}).get(str(sample.zone_token), [])
        zone = names[0] if len(names) == 1 else ''
        parent = self.data.get('maps', {}).get(str(sample.ui_map), '')
        return ' / '.join(dict.fromkeys(s for s in (parent, zone) if s)) or f'区域 {sample.ui_map}'
