from typing import List


def fromList(l, parent=None):
    key = l[0]
    value = l[1]
    children = l[2:]
    new_node = Tree(parent, key, value)
    for child in children:
        new_node.add_child(fromList(child, new_node))
    return new_node


class Tree:
    def __init__(self, parent, name, value):
        if parent is None:
            self.parent = self
        else:
            self.parent = parent
        self.name = name
        self.value = value
        self.children = []

    @staticmethod
    def fromList(l, parent=None):
        def expand_tuple(tup):
            return tup[0] if len(tup) == 1 else tup

        if type(l) is tuple:
            new_node = Tree(parent, l[0], expand_tuple(l[1:]))
        elif type(l) is list:
            key = l[0]
            if type(key) is tuple:
                new_node = Tree(parent, key[0], expand_tuple(key[1:]))
            else:
                new_node = Tree(parent, key, None)
            for elem in l[1:]:
                new_node.add_child(Tree.fromList(elem, new_node))
        return new_node

    def add_child(self, node):
        self.children.append(node)

    def has_children(self):
        return len(self.children) > 0

    def get_children(self):
        return self.children

    def get_child_by_name(self, name: str, recursive=False):
        for child in self.children:
            if child.name == name:
                return child
            if recursive:
                subchild = child.get_child_by_name(name, recursive)
                if subchild:
                    return subchild
        return None

    def get_parent(self):
        return self.parent

    def get_name(self):
        return self.name

    def get_value(self):
        return self.value

    def print_tree(self, indent=0):
        print('{}{}'.format('| ' * indent, self.name))
        for child in self.children:
            child.print_tree(indent=indent + 1)

    def __str__(self):
        return str(self.value)
