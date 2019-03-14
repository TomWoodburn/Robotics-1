**************************
Editing a ReadTheDocs file
**************************

Instructions for group members editing and contributing to an existing ReadTheDocs structure 

Installing Required Software
============================

In anaconda prompt: 

.. code-block:: bash

    conda install sphinx
    conda install -c anaconda sphinx_rtd_theme 


Use github desktop to clone robotics 1 repository 
*Pull latest version!*

Using Sphinx to Convert to Html
===============================

Use anaconda command prompt to navigate to your /Robotics-1/docs folder 

For *Windows*

.. code-block:: bash

    cd # Change directory
    dir # List contents


For *Unix operating systems*

.. code-block:: bash

    cd # Change directory
    ls # List contents


Check its clean by running:

.. code-block:: bash

    make clean

Turn .rst files into readable html files:

.. code-block:: bash

    make html

Navigate to /Robotics-1/docs/source in file explorer/finder
Double click index.html and open in chrome 
View your website!

Editing .rst files 
===================

Copy and rename an existing .rst file in the source folder to make your new file 
Make your changes 
Save file

In anaconda prompt:

.. code-block:: bash

    make clean && make html

Open the new html files as above

.. note:: Edit one file per person at a time! This is not collaborative! 

Pushing Files Once Finished 

Run : 

.. code-block:: bash   

    make clean


.. note:: Make sure to commit your rst file only!!! If you see 50+ files you have not run make clean
