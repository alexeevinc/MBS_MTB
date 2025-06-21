import numpy as np
import tomli

from src.Dataclasses.config_params import config, plot_settings

def load_config():
    try:
        with open('setup/config.toml', 'rb') as fh:
            file = tomli.load(fh)
    except FileNotFoundError:
        raise FileNotFoundError('Config file does not exist!') from None
    config_a = config(**file)

    return config_a

def load_plot_settings():
    try:
        with open('setup/plot_settings.toml', 'rb') as fh:
            file = tomli.load(fh)
    except FileNotFoundError:
        raise FileNotFoundError('Plot settings file does not exist!') from None
    config_a = plot_settings(**file)

    return config_a

