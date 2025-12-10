#!/bin/bash
# Setup script for the RAG Chatbot backend

echo "Setting up the RAG Chatbot backend environment..."

# Create virtual environment
python -m venv venv

# Activate virtual environment
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Upgrade pip
pip install --upgrade pip

# Install dependencies from requirements.txt
pip install -r requirements.txt

echo "Backend environment setup complete!"
echo "To activate the virtual environment in the future, run: source venv/bin/activate"