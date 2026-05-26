import importlib
import pkgutil

_discovered = False


def autodiscover():
    global _discovered
    if _discovered:
        return
    for m in pkgutil.walk_packages(__path__, prefix=__name__ + "."):
        importlib.import_module(m.name)
    _discovered = True
