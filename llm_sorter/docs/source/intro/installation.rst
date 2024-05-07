Installation
============

#. Download repository:

    .. code:: bash

        git clone repository_name

#. Create a virtual environment:

    .. code:: bash
        export PYTHON_KEYRING_BACKEND=keyring.backends.null.Keyring
        poetry install
