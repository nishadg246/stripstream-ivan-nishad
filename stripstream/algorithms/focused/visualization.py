from stripstream.algorithms.focused.utils import AbstractConstant

def visualize_order(streams, edges, filename='directed_graph.pdf'):
  from pygraphviz import AGraph
  import subprocess
  graph = AGraph(strict=True, directed=True)
  graph.node_attr['style'] = 'filled'
  graph.node_attr['shape'] = 'box'
  graph.node_attr['color'] = 'LightSalmon'

  def get_name(stream):
    return '\n'.join([stream.cond_stream.name,
                      ' '.join(str(inp) for inp in stream.inputs),
                      ' '.join(str(inp) for inp in stream.abs_outputs)])

  for stream in streams:
    graph.add_node(get_name(stream))
  for stream1, stream2 in edges:
    graph.add_edge(get_name(stream1), get_name(stream2))

  graph.draw(filename, prog='dot')
  subprocess.Popen('open %s'%filename, shell=True)
  raw_input('Display?')
  return graph

def visualize_streams(goals, constants, filename='graph.pdf'):
  from pygraphviz import AGraph
  import subprocess

  graph = AGraph(strict=True, directed=False)

  graph.node_attr['style'] = 'filled'
  graph.node_attr['shape'] = 'circle'
  graph.node_attr['fontcolor'] = 'black'
  graph.node_attr['colorscheme'] = 'SVG'
  graph.edge_attr['colorscheme'] = 'SVG'

  for constant in constants:
    graph.add_node(str(constant), shape='box', color='LightGreen')

  for goal in goals:
    name = '\n'.join(str(arg) for arg in [goal.predicate.name] + list(goal.args))
    #name = str(goal)
    graph.add_node(name, shape='box', color='LightBlue')
    for arg in goal.args:
      if isinstance(arg, AbstractConstant):
        arg_name = str(arg)
        graph.add_edge(name, arg_name)

  #import os
  #import webbrowser
  graph.draw(filename, prog='dot')
  subprocess.Popen('open %s'%filename, shell=True)
  #os.system(filename)
  #webbrowser.open(filename)
  raw_input('Display?')
  #safe_remove(filename)
  return graph