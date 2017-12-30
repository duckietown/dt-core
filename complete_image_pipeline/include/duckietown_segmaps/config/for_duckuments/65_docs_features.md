# Basic Markduck guide {#documentation-manual status=ready}

The Duckiebook is written in Markduck, a Markdown dialect.

It supports many features that make it possible to create
publication-worthy materials.


## Markdown

The Duckiebook is written in a Markdown dialect.

See: [A tutorial on Markdown][tutorial].

[tutorial]: https://www.markdowntutorial.com/



## Variables in command lines and command output

Use the syntax "<code><span>!</span>[name]</code>" for describing the variables in the code.

<div class="example-usage" markdown="1">

For example, to obtain:

    $ ssh ![robot name].local

Use the following:

<pre trim="1">
<code trim="1">
For example, to obtain:

    &#36; ssh <span>!</span>[robot name].local

</code>
</pre>

</div>

Make sure to quote (with 4 spaces) all command lines. Otherwise, the dollar symbol confuses the
LaTeX interpreter.



## Character escapes

Use the string <q><code>&amp;#36;</code></q> to write the dollar symbol
<q><code>&#36;</code></q>, otherwise it gets confused with LaTeX math materials. Also notice
that you should probably use "USD" to refer to U.S. dollars.

Other symbols to escape are shown in [](#tab:escapes).

<col2 figure-id="tab:escapes" figure-caption="Symbols to escape">
    <s>use <code>&amp;#36;</code> </s> <s>instead of <code>&#36;</code></s>
    <s>use <code>&amp;#96;</code> </s> <s>instead of <code>&#96;</code></s>
    <s>use <code>&amp;#lt;</code> </s> <s>instead of <code>&lt;</code></s>
    <s>use <code>&amp;#gt;</code> </s> <s>instead of <code>&gt;</code></s>
</col2>


## Keyboard keys

Use the `kbd` element for keystrokes.

<div class="example-usage" markdown="1">

For example, to obtain:

> Press <kbd>a</kbd> then <kbd>Ctrl</kbd>-<kbd>C</kbd>.

use the following:

    Press <kbd>a</kbd> then <kbd>Ctrl</kbd>-<kbd>C</kbd>.

</div>

## Figures {#figures}

For any element, adding an attribute called `figure-id`
with value `fig:![figure ID]` or `tab:![table ID]`
will create a figure that wraps the element.


For example:

    <div figure-id="fig:![figure ID]">
        ![figure content]
    </div>

It will create HMTL of the form:

    <div id='fig:code-wrap' class='generated-figure-wrap'>
        <figure id='fig:![figure ID]' class='generated-figure'>
            <div>
                ![figure content]
            </div>
        </figure>
    </div>

<!--
To add a class to the figure, use `figure-class`:

    <div figure-id="fig:code" figure-class="myclass">
        ![figure content]
    </div>

This will give it to the <code>&lt;figure&gt;</code> and the containing <code>&lt;figure&gt;</code>


Useful classes:

* `float_bottom`

-->

To add a caption, add an attribute `figure-caption`:

    <div figure-id="fig:![figure ID]" figure-caption="This is my caption">
        ![figure content]
    </div>

Alternatively, you can put anywhere an element `figcaption` with ID `![figure id]:caption`:

    <element figure-id="fig:![figure ID]">
        ![figure content]
    </element>

    <figcaption id='fig:![figure ID]:caption'>
        This the caption figure.
    </figcaption>

To refer to the figure, use an empty link:

    Please see [](#fig:![figure ID]).

The code will put a reference to "Figure XX".


## Subfigures

You can also create subfigures, using the following syntax.

```html
<div figure-id="fig:big">
    <figcaption>Caption of big figure</figcaption>

    <div figure-id="subfig:first" figure-caption="Caption 1">
        <p style='width:5em;height:5em;background-color:#eef'>first subfig</p>
    </div>

    <div figure-id="subfig:second" figure-caption="Caption 2">
        <p style='width:5em;height:5em;background-color:#fee'>second subfig</p>
    </div>
</div>
```

This is the result:

<div figure-id="fig:big">
    <figcaption>Caption of big figure</figcaption>

    <div figure-id="subfig:first" figure-caption="Caption 1">
        <p style='width:5em;height:5em;background-color:#eef'>first subfig</p>
    </div>

    <div figure-id="subfig:second" figure-caption="Caption 2">
        <p style='width:5em;height:5em;background-color:#fee'>second subfig</p>
    </div>
</div>

By default, the subfigures are displayed one per line.

To make them flow horizontally, add `figure-class="flow-subfigures"` to the external figure `div`. Example:

<div figure-id="fig:big2" figure-class="flow-subfigures">
    <figcaption>Caption of big figure</figcaption>

    <div figure-id="subfig:first2" figure-caption="Caption 1">
        <p style='width:5em;height:5em;background-color:#eef'>first subfig</p>
    </div>

    <div figure-id="subfig:second2" figure-caption="Caption 2">
        <p style='width:5em;height:5em;background-color:#fee'>second subfig</p>
    </div>
</div>


## Shortcut for tables

The shortcuts `col2`, `col3`, `col4`, `col5`
are expanded in tables with 2, 3, 4 or 5 columns.

The following code:

<pre trim="1">
<code trim="1">
&lt;col2 figure-id="tab:mytable" figure-caption="My table"&gt;
    &lt;span&gt;A&lt;/span&gt;
    &lt;span&gt;B&lt;/span&gt;
    &lt;span&gt;C&lt;/span&gt;
    &lt;span&gt;D&lt;/span&gt;
&lt;/col2&gt;
</code>
</pre>

gives the following result:

<col2 figure-id="tab:mytable" figure-caption="My table">
    <span>A</span>
    <span>B</span>
    <span>C</span>
    <span>D</span>
</col2>

### `labels-row1`  and `labels-row1`

Use the classes `labels-row1`  and `labels-row1` to make pretty tables like the following.

`labels-row1`: the first row is the headers.

`labels-col1`: the first column is the headers.

<col4 figure-id="tab:mytable-col1" class="labels-col1">
    <figcaption>Using <code>class="labels-col1"</code></figcaption>
    <span>header A </span>
    <span>B</span>
    <span>C</span>
    <span>1</span>
    <span>header D</span>
    <span>E</span>
    <span>F</span>
    <span>2</span>
    <span>header G</span>
    <span>H</span>
    <span>I</span>
    <span>3</span>
</col4>

<col3 figure-id="tab:mytable-row1" class="labels-row1">
    <figcaption>Using <code>class="labels-row1"</code></figcaption>
    <span>header A</span>
    <span>header B</span>
    <span>header C</span>
    <span>D</span>
    <span>E</span>
    <span>F</span>
    <span>G</span>
    <span>H</span>
    <span>I</span>
    <span>1</span>
    <span>2</span>
    <span>3</span>
</col3>

## Linking to documentation {#linking-to-documentation}

### Establishing names of headers {#establishing}

You give IDs to headers using the format:

    ### ![header title] {#![topic ID]}

For example, for this subsection, we have used:

    ### Establishing names of headers {#establishing}

With this, we have given this header the ID "`establishing`".


### How to name IDs - and why it's not automated

Some time ago, if there was a section called

    ## My section

then it would be assigned the ID "my-section".

This behavior has been removed, for several reasons.

One is that if you don't see the ID then you will be tempted to just change the name:

    ## My better section

and silently the ID will be changed to "my-better-section" and all the previous links will be invalidated.

The current behavior is to generate an ugly link like "autoid-209u31j".

This will make it clear that you cannot link using the PURL if you don't assign an ID.


Also, I would like to clarify that all IDs are *global* (so it's easy to link stuff, without thinking about namespaces, etc.).

Therefore, please choose descriptive IDs, with at least two IDs.

E.g. if you make a section called

    ## Localization  {#localization}

that's certainly a no-no, because "localization" is too generic.

Better:

    ## Localization {#intro-localization}

Also note that you don't *need* to add IDs to everything, only the things that people could link to. (e.g. not subsubsections)


### Linking from the documentation to the documentation

You can use the syntax:

    [](#![topic ID])

to refer to the header.

You can also use some slightly more complex syntax that also allows
to link to only the name, only the number or both ([](#tab:link-examples)).

<col1 figure-id="tab:link-examples" figure-caption="Syntax for referring to sections.">
    <pre><code>See [](#establishing).</code></pre>
    <s>See <a href="#establishing"></a></s>
    <pre><code>See &lt;a class="only_name" href="#establishing"&gt;&lt;/a&gt;.</code></pre>
    <s>See <a class="only_name" href="#establishing"></a>.</s>
    <pre><code>See &lt;a class="only_number" href="#establishing"&gt;&lt;/a&gt;.</code></pre>
    <s>See <a class="only_number" href="#establishing"></a>.</s>
    <pre><code>See &lt;a class="number_name" href="#establishing"&gt;&lt;/a&gt;.</code></pre>
    <s>See <a class="number_name" href="#establishing"></a>.</s>
</col1>

<style>
#tab\:link-examples td {
    text-align: left;
    display: block;
    margin-bottom: 5pt;
}

#tab\:link-examples tr:nth-child(2n+1) td {
    margin-bottom: 5pt;
}
#tab\:link-examples tr:nth-child(2n) td {
    margin-bottom: 15pt;
}
</style>

### Linking to the documentation from outside the documentation

You are encouraged to put links to the documentation from the code or scripts.

To do so, use links of the form:

    http://purl.org/dth/![topic ID]

Here "`dth`" stands for "Duckietown Help". This link will get redirected to
the corresponding document on the website.

For example, you might have a script whose output is:

    $ rosrun mypackage myscript
    Error. I cannot find the scuderia file.
    See: http://purl.org/dth/scuderia

When the user clicks on the link, they will be redirected to [](#scuderia).
