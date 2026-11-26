"""Generate the Gazebo viewport meshes from the licensed full-resolution STL set."""
from pathlib import Path
import argparse

import vtk


def simplify(source, target):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(source))
    reader.Update()
    triangles = reader.GetOutput().GetNumberOfCells()
    decimator = vtk.vtkQuadricDecimation()
    decimator.SetInputConnection(reader.GetOutputPort())
    decimator.SetTargetReduction(max(0.0, 1.0 - 3500 / triangles))
    writer = vtk.vtkSTLWriter()
    writer.SetInputConnection(decimator.GetOutputPort())
    writer.SetFileTypeToBinary()
    writer.SetFileName(str(target))
    if not writer.Write():
        raise RuntimeError(f'Could not write {target}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for mesh in sorted(args.source.glob('*.STL')):
        simplify(mesh, args.output / mesh.name)
