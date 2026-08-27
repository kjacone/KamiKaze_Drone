#!/usr/bin/env python3
# drone_control/scripts/generate_schema_doc.py

import yaml
import json
import os
import argparse
from typing import Dict, Any

class SchemaDocumentationGenerator:
    """Generate comprehensive documentation from parameter schemas"""
    
    def __init__(self, config_dir: str = "/drone_control/config"):
        self.config_dir = config_dir
        self.output_dir = os.path.join(config_dir, "docs")
        os.makedirs(self.output_dir, exist_ok=True)
        
    def generate_all_documentation(self):
        """Generate documentation for all config files"""
        docs = {}
        
        config_files = [
            'target_params.yaml',
            'camera_calib.yaml',
            'flight_control.yaml',
            'kalman_filter.yaml',
            'system_params.yaml'
        ]
        
        for config_file in config_files:
            if os.path.exists(os.path.join(self.config_dir, config_file)):
                docs[config_file] = self.generate_documentation(config_file)
                
        # Generate index and combined documentation
        self.generate_index(docs)
        self.generate_combined_documentation(docs)
        self.generate_markdown_documentation()
        
    def generate_documentation(self, config_file: str) -> Dict[str, Any]:
        """Generate documentation for a single config file"""
        file_path = os.path.join(self.config_dir, config_file)
        
        with open(file_path, 'r') as f:
            config = yaml.safe_load(f)
            
        doc = {
            "filename": config_file,
            "description": f"Configuration for {config_file}",
            "parameters": self._extract_parameters(config, config_file)
        }
        
        # Save individual documentation
        doc_file = os.path.join(
            self.output_dir, 
            os.path.splitext(config_file)[0] + "_docs.yaml"
        )
        with open(doc_file, 'w') as f:
            yaml.dump(doc, f, default_flow_style=False)
            
        return doc
        
    def _extract_parameters(self, config: Dict, config_file: str) -> Dict:
        """Extract and document all parameters"""
        params = {}
        flat_params = self._flatten_dict(config)
        
        for key, value in flat_params.items():
            param_info = {
                "path": key,
                "type": type(value).__name__,
                "value": value,
                "description": f"Parameter {key} from {config_file}"
            }
            params[key] = param_info
            
        return params
        
    def _flatten_dict(self, d: Dict, parent_key: str = "", sep: str = '.') -> Dict:
        """Flatten nested dictionary with dot notation"""
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            
            if isinstance(v, dict):
                items.extend(self._flatten_dict(v, new_key, sep=sep).items())
            else:
                items.append((new_key, v))
                
        return dict(items)
        
    def generate_index(self, docs: Dict):
        """Generate index of all configuration files"""
        index = {
            "generated": "2024-01-01",
            "total_files": len(docs),
            "configs": []
        }
        
        for filename, doc in docs.items():
            config_info = {
                "filename": filename,
                "description": doc.get("description", ""),
                "parameters": len(doc.get("parameters", {})),
                "path": f"config/{filename}"
            }
            index["configs"].append(config_info)
            
        index_file = os.path.join(self.output_dir, "index.yaml")
        with open(index_file, 'w') as f:
            yaml.dump(index, f, default_flow_style=False)
            
    def generate_combined_documentation(self, docs: Dict):
        """Generate combined JSON documentation"""
        combined = {
            "version": "1.0",
            "timestamp": "2024-01-01",
            "configurations": docs
        }
        
        json_file = os.path.join(self.output_dir, "combined_schema.json")
        with open(json_file, 'w') as f:
            json.dump(combined, f, indent=2)
            
    def generate_markdown_documentation(self):
        """Generate Markdown documentation for human reading"""
        md_content = "# Drone Control System Configuration Documentation\n\n"
        
        config_files = [
            'target_params.yaml',
            'camera_calib.yaml',
            'flight_control.yaml',
            'kalman_filter.yaml',
            'system_params.yaml'
        ]
        
        for config_file in config_files:
            doc_file = os.path.join(
                self.output_dir,
                os.path.splitext(config_file)[0] + "_docs.yaml"
            )
            
            if os.path.exists(doc_file):
                with open(doc_file, 'r') as f:
                    doc = yaml.safe_load(f)
                    
                md_content += f"## {config_file}\n\n"
                md_content += f"{doc.get('description', '')}\n\n"
                
                md_content += "### Parameters\n\n"
                md_content += "| Parameter | Type | Value | Description |\n"
                md_content += "|-----------|------|-------|-------------|\n"
                
                for param_name, param_info in doc.get('parameters', {}).items():
                    md_content += f"| {param_name} | {param_info['type']} | {param_info['value']} | {param_info['description']} |\n"
                    
                md_content += "\n"
                
        # Save markdown file
        md_file = os.path.join(self.output_dir, "parameter_reference.md")
        with open(md_file, 'w') as f:
            f.write(md_content)
            
        print(f"Documentation generated in {self.output_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate parameter documentation')
    parser.add_argument('--config-dir', default='/drone_control/config', 
                       help='Configuration directory')
    
    args = parser.parse_args()
    
    generator = SchemaDocumentationGenerator(args.config_dir)
    generator.generate_all_documentation()
    
    print(f"Documentation generated in {generator.output_dir}")