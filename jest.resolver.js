// The @actions/* packages are ESM-only and only define the "import" export
// condition, so add it when resolving them. Jest's CommonJS module resolution
// does not use that condition by default.
module.exports = (path, options) =>
	options.defaultResolver(path, {
		...options,
		conditions: path.startsWith("@actions/")
			? [...(options.conditions ?? []), "import"]
			: options.conditions,
	});
