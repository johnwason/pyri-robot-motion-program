from pyri.plugins.webui_server import PyriWebUIServerPluginFactory
from pyri.webui_server.webui_resource_router import PyriWebUIResourceRouteHandler

class PyriRoboticsMPWebUIServerPluginFactory(PyriWebUIServerPluginFactory):
    def get_plugin_name(self):
        return "pyri-robotics-motion-program"

    def get_plugin_route_handler(self):
        return PyriWebUIResourceRouteHandler(__package__).handler

def get_webui_factory():
    return PyriRoboticsMPWebUIServerPluginFactory()
