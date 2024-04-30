class SceneGraphNode:
    def __init__(self, name = "", attributes = None):
        if attributes:
            self.attributes = attributes
        else:
            self.attributes = []
        self.name = name
    
    def add_attribute(self, attribute):
        self.attributes.append(attribute)
 
    def remove_attribute(self, attribute):
        self.attributes.remove(attribute)
 
    def clear_attributes(self):
        self.attributes.clear()
 
    def get_attributes(self):
        return self.attributes
    
    def get_name(self):
        return self.name
    
    def __str__(self):
        return f"SceneGraphNode(name={self.name}, attributes={self.attributes})"