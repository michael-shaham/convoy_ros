import torch.nn as nn


class FullyConnected(nn.Module):
    def __init__(self, input_size: int, output_size: int, 
                 layer_sizes: list):
        super(FullyConnected, self).__init__()
        layers = [nn.Linear(input_size, layer_sizes[0])]
        layers += [nn.ReLU(True)]
        for i in range(1, len(layer_sizes[1:])):
            layers += [nn.Linear(layer_sizes[i-1], layer_sizes[i])] 
            layers += [nn.ReLU(True)]
        layers += [nn.Linear(layer_sizes[-1], output_size)]
        self.model = nn.Sequential(*layers)

    def forward(self, x):
        logits = self.model(x)
        return logits
