"""Vehicle-VM diagnostic registration via package.preload; no game-file edits."""
import json
from pathlib import Path
import re

NAME = 'wildosContactAudit20261018'
PROTOCOL = 'wildos_contact_audit_v1'


def literal(instance):
    if not isinstance(instance, str) or not re.fullmatch(r'[a-z0-9_]{1,64}', instance):
        raise ValueError('Invalid diagnostic instance identifier')
    return json.dumps(instance)


def install(instance):
    ident = literal(instance)
    source = Path(__file__).with_name('contact_observer.lua').read_text()
    return f"""
local name={json.dumps(NAME)}
local key='controller/'..name
assert(controller.getController(name)==nil, 'Diagnostic controller already present')
assert(package.preload[key]==nil and package.loaded[key]==nil, 'Diagnostic module already present')
package.preload[key]=function()
{source}
end
local c=controller.loadControllerExternal(name,name,{{instance_id={ident},manualOrder=2000000000}})
assert(c and c.protocol=={json.dumps(PROTOCOL)} and c.instance_id=={ident}, 'Diagnostic load failed')
return jsonEncode({{protocol=c.protocol,instance_id=c.instance_id,installed=true}})
"""


def snapshot(instance):
    ident = literal(instance)
    return f"""
local c=assert(controller.getController({json.dumps(NAME)}),'Diagnostic controller missing')
assert(c.protocol=={json.dumps(PROTOCOL)} and c.instance_id=={ident},'Diagnostic owner mismatch')
return jsonEncode(c.snapshot())
"""


def uninstall(instance):
    ident = literal(instance)
    return f"""
local name={json.dumps(NAME)}
local key='controller/'..name
local c=controller.getController(name)
if c then
  assert(c.protocol=={json.dumps(PROTOCOL)} and c.instance_id=={ident},'Diagnostic owner mismatch')
  controller.unloadControllerExternal(name)
end
assert(controller.getController(name)==nil,'Diagnostic unload failed')
package.loaded[key]=nil
package.preload[key]=nil
return jsonEncode({{instance_id={ident},unloaded=true}})
"""
