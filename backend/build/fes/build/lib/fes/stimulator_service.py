from loguru import logger


class StimulatorService:
    def __init__(self, port_name: str):
        self.rehamove = None

        if port_name == "":
            logger.error("No USB port supplied for the stimulator")
        self.rehamove = self.connect_to_rehamove(port_name)

    @staticmethod
    def connect_to_rehamove(port_name: str) -> Rehamove:
        rehamove = None
        while rehamove is None:  # continuously connect
            rehamove = Rehamove(port_name)
        return rehamove

    def mainloop(self):
        pass
